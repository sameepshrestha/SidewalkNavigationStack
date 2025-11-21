import asyncio
import json
import logging
import aiohttp
import cv2
import numpy as np
import threading
import time
from . import robot_messages_pb2
import queue
from aiortc import RTCPeerConnection, RTCSessionDescription
# import robot_messages_pb2

SERVER_OFFER_URL = "http://100.102.181.1:8080/offer" 

logging.basicConfig(level=logging.INFO)

class Robotclient:
    def __init__(self, signaling_url):
        self._server_url = signaling_url
        self.pc = RTCPeerConnection()
        self._frames = {"front": None, "back": None, "left": None, "right": None}
        self._frames_lock = threading.Lock()
        self.latest_sensor_data = None
        self._sensor_lock = threading.Lock()

        self.mid_map = {"0": "front", "1": "back", "2": "left", "3": "right"}
        self.logger = logging.getLogger("robotrtclient")
        self.connection_ready = threading.Event()
        self._loop = None
        self._thread = None
        self._keep_alive = True
        # This internal asyncio event is used within the network thread
        self._command_queue = queue.Queue()
        self._data_channel_ready_async = None 

    async def _data_channel_setup(self):
        self._data_channel_ready_async = asyncio.Event()
        self.dc = self.pc.createDataChannel("protobuf")
        #self.logger.info("Client: created data channel 'protobuf'")

        @self.dc.on("open")
        def on_open():
           # self.logger.info("✅ Data channel open")
            self._data_channel_ready_async.set() # Set the asyncio event
            self.connection_ready.set() # Signal the main thread

        @self.dc.on("message")
        def on_message(message):
            try:
                sd = robot_messages_pb2.SensorData()
                sd.ParseFromString(message)
                # print a small summary
                print(f"[SensorData] seq={sd.sequence} timestamp={sd.timestamp} yaw={sd.imu.yaw:.2f}")
            except Exception:
                try:
                    rs = robot_messages_pb2.RobotStatus()
                    rs.ParseFromString(message)
                    #print(f"[RobotStatus] status parsed: {rs}")
                except Exception:
                    print(f"[DataChannel] Received raw message len={len(message) if hasattr(message,'__len__') else 'unknown'}")
                pass

    async def _video_setup(self):
        self.track_count = 0 
        for _ in range(3):
            self.pc.addTransceiver("video", direction="recvonly")
        async def on_track(track):

            label = self.mid_map.get(str(self.track_count))
            if label:
                asyncio.create_task(self._handle_track(track, label))
            self.track_count += 1
        self.pc.on("track", on_track)
    async def _handle_track(self, track, label):
        self.logger.info(f"Processing track {label} (kind={track.kind})")
        if track.kind == "video":
            while True:
                try:
                    frame = await track.recv()
                    img = frame.to_ndarray(format="bgr24")
                    # self.logger.info(f"Received frame for {label}")
                    with self._frames_lock:
                        self._frames[label] = img
                except Exception as e:
                    self.logger.error(f"Track {label} error: {e}")
                    break
    async def _send_command(self, steering, throttle):
        if self.dc and self.dc.readyState == "open":
            cmd = robot_messages_pb2.Command()
            cmd.steering = steering
            cmd.throttle = throttle
            buffer = cmd.SerializeToString()
            self.dc.send(buffer)
           # self.logger.info(f"Sent Command: steering={steering}, throttle={throttle}")
        else:
            self.logger.warning("Data channel not open, cannot send command")

    async def _process_command_queue(self):
        while self._keep_alive:
            try:
                steering, throttle = self._command_queue.get_nowait()
                await self._send_command(steering, throttle)
                self._command_queue.task_done()
            except queue.Empty:
                await asyncio.sleep(0.01)
    def send_command(self, steering, throttle):
        """Synchronous method to queue a command for sending."""
        self._command_queue.put((steering, throttle))
        self.logger.info(f"Queued Command: steering={steering}, throttle={throttle}")

    async def _connect_and_run(self):
        self._loop = asyncio.get_running_loop()
        await self._data_channel_setup()
        await self._video_setup()
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        
        try:
            async with aiohttp.ClientSession() as session:
                payload = {"sdp": self.pc.localDescription.sdp, "type": self.pc.localDescription.type}
                self.logger.info("Signal: posting offer...")
                async with session.post(self._server_url, json=payload) as resp:
                    if resp.status != 200: raise Exception(f"Server returned {resp.status}")
                    answer = await resp.json()
                    self.logger.info(f"Remote SDP: {answer['sdp']}")
            await self.pc.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))
            
            await asyncio.wait_for(self._data_channel_ready_async.wait(), timeout=20)
            asyncio.create_task(self._process_command_queue())
            await asyncio.Event().wait() # Keep the connection alive
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            if not self.connection_ready.is_set():
                self.connection_ready.set() # Signal main thread on failure too
        finally:
            if self.pc.connectionState != "closed": await self.pc.close()

    def start(self):
        self._thread = threading.Thread(target=self._run_asyncio_loop, daemon=True)
        self._thread.start()
            
    def _run_asyncio_loop(self):
        self._keep_alive =- False
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._connect_and_run())
        finally:
            self._loop.close()
    def stop(self):
        self.logger.info("Stopping robot client...")
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=2)

    @property
    def front_camera_frame(self):
        with self._frames_lock:
            frame = self._frames.get("front")
            return frame.copy() if frame is not None else None

    @property
    def left_camera_frame(self):
        with self._frames_lock:
            frame = self._frames.get("left")
            return frame.copy() if frame is not None else None

    @property
    def right_camera_frame(self):
        with self._frames_lock:
            frame = self._frames.get("right")
            return frame.copy() if frame is not None else None
