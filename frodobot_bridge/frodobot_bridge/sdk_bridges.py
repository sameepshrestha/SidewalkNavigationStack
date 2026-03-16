import base64
import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import requests
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, MagneticField, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8


def unix_to_stamp(node: Node, ts: Optional[float]) -> Time:
    now = node.get_clock().now().to_msg()
    if ts is None or float(ts) <= 0.0:
        return now
    sec = int(float(ts))
    nanosec = int((float(ts) - sec) * 1e9)
    stamp = Time()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def decode_base64_image(payload: str) -> np.ndarray:
    data = base64.b64decode(payload)
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode image payload")
    return img


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def pick_ts(*samples: Optional[Sequence[float]], fallback: Optional[float] = None) -> Optional[float]:
    for sample in samples:
        if sample is not None and len(sample) > 3:
            return float(sample[3])
    return fallback


def zip_longest_safe(a: Sequence[Any], b: Sequence[Any]) -> Iterable[Tuple[Optional[Any], Optional[Any]]]:
    max_len = max(len(a), len(b))
    for i in range(max_len):
        yield (a[i] if i < len(a) else None, b[i] if i < len(b) else None)


class EarthRoverBridge(Node):
    """
    Earth Rover SDK bridge for ROS 2.

    Notes for FrodoBots IMU:
    - Accelerometer units from SDK are in g
    - Gyroscope units from SDK are in deg/s
    - Magnetometer is passed through in raw units after axis remap
    - IMU data is remapped into ROS body frame before publication

    Body frame convention:
      +X forward
      +Y left
      +Z up

    Provided calibration:
      +X_body = -Z_imu
      +Y_body = +Y_imu
      +Z_body = +X_imu

    So we publish imu/data_raw already aligned to imu_link/body axes.
    """

    def __init__(self) -> None:
        super().__init__("earth_rover_sdk_bridge")

        # SDK 
        self.declare_parameter("sdk_base_url", "http://127.0.0.1:8000")
        self.declare_parameter("request_timeout_s", 2.0)
        self.declare_parameter("verify_ssl", True)
        self.declare_parameter("image_mode", "paired")  # paired/front/rear
        self.declare_parameter("image_rate_hz", 5.0)
        self.declare_parameter("data_rate_hz", 10.0)
        # Frames
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("front_camera_frame", "front_camera_optical_frame")
        self.declare_parameter("rear_camera_frame", "rear_camera_optical_frame")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("gps_frame", "gps_link")
        # Motion / status interpretation
        self.declare_parameter("assume_orientation_is_degrees", True)
        self.declare_parameter("rpm_joint_names", ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"])
        self.declare_parameter("rpm_to_rad_s", 2.0 * math.pi / 60.0)
        self.declare_parameter("publish_joint_positions", False)
        # Wheel odometry parameters (differential drive)
        # From URDF: wheel_radius=0.055m, body_width=0.273m
        self.declare_parameter("wheel_radius_m", 0.055)
        self.declare_parameter("wheel_track_m", 0.273)   # distance between left and right wheels
        self.declare_parameter("publish_wheel_odom", True)
        # Camera info placeholders. Replace with calibrated values later.
        self.declare_parameter("front_camera_info.width", 1024)
        self.declare_parameter("front_camera_info.height", 576)
        self.declare_parameter("rear_camera_info.width", 540)
        self.declare_parameter("rear_camera_info.height", 360)
        self.declare_parameter("front_camera_info.k", [0.0] * 9)
        self.declare_parameter("rear_camera_info.k", [0.0] * 9)
        self.declare_parameter("front_camera_info.d", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("rear_camera_info.d", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("camera_distortion_model", "plumb_bob")
        # Covariances
        self.declare_parameter("imu_linear_accel_cov", [0.2, 0.2, 0.2])
        self.declare_parameter("imu_angular_vel_cov", [0.05, 0.05, 0.05])
        self.declare_parameter("mag_cov", [0.5, 0.5, 0.5])

        # FrodoBots IMU conversion / calibration
        self.declare_parameter("accel_scale_to_mps2", 9.81)
        self.declare_parameter("gyro_scale_to_rads", math.pi / 180.0)
        self.declare_parameter("gyro_bias_deg_s", [0.0, 0.0, 0.0])  # overridden by auto-cal
        self.declare_parameter("apply_imu_axis_remap", True)
        # Auto-calibrate gyro bias at startup (robot must be still for first N seconds)
        self.declare_parameter("gyro_auto_calibrate", True)
        self.declare_parameter("gyro_cal_duration_s", 20.0)
        # Magnetic field scaling, if you later want SI Tesla:
        # Tesla = raw_value * magnetometer_scale_to_tesla
        # Leave as 1.0 for now if raw calibration is unknown.
        self.declare_parameter("magnetometer_scale_to_tesla", 1.0)
        self.base_url = self.get_parameter("sdk_base_url").value.rstrip("/")
        self.request_timeout_s = float(self.get_parameter("request_timeout_s").value)
        self.verify_ssl = bool(self.get_parameter("verify_ssl").value)
        self.image_mode = str(self.get_parameter("image_mode").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.front_camera_frame = str(self.get_parameter("front_camera_frame").value)
        self.rear_camera_frame = str(self.get_parameter("rear_camera_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.gps_frame = str(self.get_parameter("gps_frame").value)

        self.rpm_joint_names = list(self.get_parameter("rpm_joint_names").value)
        self.rpm_to_rad_s = float(self.get_parameter("rpm_to_rad_s").value)
        self.assume_orientation_is_degrees = bool(self.get_parameter("assume_orientation_is_degrees").value)
        self.publish_joint_positions = bool(self.get_parameter("publish_joint_positions").value)
        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_track_m  = float(self.get_parameter("wheel_track_m").value)
        self.publish_wheel_odom = bool(self.get_parameter("publish_wheel_odom").value)
        # Odometry integration state
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._odom_last_stamp: float = None  # unix time of last RPM sample
        self.camera_distortion_model = str(self.get_parameter("camera_distortion_model").value)
        self.imu_linear_accel_cov = [float(v) for v in self.get_parameter("imu_linear_accel_cov").value]
        self.imu_angular_vel_cov = [float(v) for v in self.get_parameter("imu_angular_vel_cov").value]
        self.mag_cov = [float(v) for v in self.get_parameter("mag_cov").value]

        self.accel_scale_to_mps2 = float(self.get_parameter("accel_scale_to_mps2").value)
        self.gyro_scale_to_rads = float(self.get_parameter("gyro_scale_to_rads").value)
        self.gyro_bias_deg_s = [float(v) for v in self.get_parameter("gyro_bias_deg_s").value]
        self.apply_imu_axis_remap = bool(self.get_parameter("apply_imu_axis_remap").value)
        #gyro calibration
        self._gyro_auto_cal = bool(self.get_parameter("gyro_auto_calibrate").value)
        self._gyro_cal_duration = float(self.get_parameter("gyro_cal_duration_s").value)
        self._gyro_cal_samples: list = []       # raw (deg/s) samples during cal
        self._gyro_cal_done = not self._gyro_auto_cal  # skip if disabled
        self._gyro_cal_start_time: float = None
        self.magnetometer_scale_to_tesla = float(self.get_parameter("magnetometer_scale_to_tesla").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.front_image_pub = self.create_publisher(Image, "/earth_rover/front/image_raw", sensor_qos)
        self.rear_image_pub = self.create_publisher(Image, "/earth_rover/rear/image_raw", sensor_qos)
        self.front_info_pub = self.create_publisher(CameraInfo, "/earth_rover/front/camera_info", 10)
        self.rear_info_pub = self.create_publisher(CameraInfo, "/earth_rover/rear/camera_info", 10)

        self.imu_pub = self.create_publisher(Imu, "/earth_rover/imu/data_raw", sensor_qos)
        self.mag_pub = self.create_publisher(MagneticField, "/earth_rover/imu/mag", sensor_qos)
        self.gps_pub = self.create_publisher(NavSatFix, "/earth_rover/gps/fix", 10)
        self.joint_pub = self.create_publisher(JointState, "/earth_rover/wheels/joint_states", 10)
        self.wheel_odom_pub = self.create_publisher(Odometry, "/wheel/odom", 10)

        self.battery_pub = self.create_publisher(UInt8, "/earth_rover/diagnostics/battery", 10)
        self.signal_pub = self.create_publisher(UInt8, "/earth_rover/diagnostics/signal_level", 10)
        self.gps_signal_pub = self.create_publisher(Float32, "/earth_rover/diagnostics/gps_signal", 10)
        self.orientation_pub = self.create_publisher(Float32, "/earth_rover/status/orientation_deg", 10)
        self.speed_pub = self.create_publisher(Float32, "/earth_rover/status/speed", 10)

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.last_cmd_vel_payload = {"command": {"linear": 0.0, "angular": 0.0, "lamp": 0}}
        self.last_cmd_vel_stamp = None
        self.create_timer(0.05, self.poll_control)  # 20Hz loop to beat SDK 100ms watchdog
        
        self.session = requests.Session()

        self.front_camera_info = self._build_camera_info("front_camera_info", self.front_camera_frame)
        self.rear_camera_info = self._build_camera_info("rear_camera_info", self.rear_camera_frame)

        self.last_joint_stamp: Optional[Time] = None
        self.last_joint_position = [0.0, 0.0, 0.0, 0.0]

        image_rate_hz = max(0.1, float(self.get_parameter("image_rate_hz").value))
        data_rate_hz = max(0.1, float(self.get_parameter("data_rate_hz").value))
        self.create_timer(1.0 / image_rate_hz, self.poll_images)
        self.create_timer(1.0 / data_rate_hz, self.poll_data)

        self.get_logger().info(f"Connecting to Earth Rover SDK at {self.base_url}")
        if self._gyro_auto_cal:
            self.get_logger().info(
                f"Gyro auto-calibration enabled: collecting for {self._gyro_cal_duration}s at startup. "
                "Keep robot STILL!")
        else:
            self.get_logger().info(
                "IMU remap enabled: "
                f"{self.apply_imu_axis_remap}, accel scale={self.accel_scale_to_mps2}, "
                f"gyro scale={self.gyro_scale_to_rads}, gyro bias={self.gyro_bias_deg_s}"
            )

    def _build_camera_info(self, prefix: str, frame_id: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = int(self.get_parameter(f"{prefix}.width").value)
        msg.height = int(self.get_parameter(f"{prefix}.height").value)
        msg.distortion_model = self.camera_distortion_model
        msg.k = [float(v) for v in self.get_parameter(f"{prefix}.k").value]
        msg.d = [float(v) for v in self.get_parameter(f"{prefix}.d").value]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        if msg.k[0] != 0.0 and msg.k[4] != 0.0:
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            msg.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
        else:
            msg.p = [0.0] * 12
        return msg

    def http_get(self, path: str) -> requests.Response:
        return self.session.get(
            f"{self.base_url}{path}",
            timeout=self.request_timeout_s,
            verify=self.verify_ssl,
        )

    def http_post(self, path: str, payload: Dict[str, Any]) -> requests.Response:
        return self.session.post(
            f"{self.base_url}{path}",
            json=payload,
            timeout=self.request_timeout_s,
            verify=self.verify_ssl,
        )
    # IMU conversion helpers

    def remap_accel_to_body(self, acc: Sequence[float]) -> Tuple[float, float, float]:
        """
        Raw accelerometer -> ROS body frame, in m/s^2.
        IMU -> Body mapping:
          +X_body = -Z_imu
          +Y_body = +Y_imu
          +Z_body = +X_imu
        """
        ax_i, ay_i, az_i = float(acc[0]), float(acc[1]), float(acc[2])

        ax_b = -az_i * self.accel_scale_to_mps2
        ay_b =  ay_i * self.accel_scale_to_mps2
        az_b =  ax_i * self.accel_scale_to_mps2
        return ax_b, ay_b, az_b

    def remap_gyro_to_body(self, gyro: Sequence[float]) -> Tuple[float, float, float]:
        """
        Raw gyroscope -> ROS body frame, in rad/s, with bias removed.
        autocalibation for gyro bias is done at startup
        Raw gyro units: deg/s
        """
        raw = [float(gyro[0]), float(gyro[1]), float(gyro[2])]

        if not self._gyro_cal_done:
            import time as _time
            now = _time.time()
            if self._gyro_cal_start_time is None:
                self._gyro_cal_start_time = now
            self._gyro_cal_samples.append(raw)
            if now - self._gyro_cal_start_time >= self._gyro_cal_duration:
                n = len(self._gyro_cal_samples)
                self.gyro_bias_deg_s = [
                    sum(s[i] for s in self._gyro_cal_samples) / n
                    for i in range(3)
                ]
                self._gyro_cal_done = True
                self.get_logger().info(
                    f"Gyro auto-cal complete ({n} samples): "
                    f"bias={[f'{b:.4f}' for b in self.gyro_bias_deg_s]} deg/s")
            return 0.0, 0.0, 0.0

        gx_i = raw[0] - self.gyro_bias_deg_s[0]
        gy_i = raw[1] - self.gyro_bias_deg_s[1]
        gz_i = raw[2] - self.gyro_bias_deg_s[2]

        gx_b = -gz_i * self.gyro_scale_to_rads
        gy_b =  gy_i * self.gyro_scale_to_rads
        gz_b =  gx_i * self.gyro_scale_to_rads
        return gx_b, gy_b, gz_b

    def remap_mag_to_body(self, mag: Sequence[float]) -> Tuple[float, float, float]:
        """
        Raw magnetometer -> ROS body frame.

        We apply only axis remap and optional scalar conversion.
        If later you obtain a proper hard/soft iron calibration,
        apply it here.
        """
        mx_i, my_i, mz_i = float(mag[0]), float(mag[1]), float(mag[2])

        mx_b = -mz_i * self.magnetometer_scale_to_tesla
        my_b =  my_i * self.magnetometer_scale_to_tesla
        mz_b =  mx_i * self.magnetometer_scale_to_tesla
        return mx_b, my_b, mz_b

    # Image polling

    def poll_images(self) -> None:
        try:
            if self.image_mode == "front":
                self._poll_single_image(
                    "/screenshot?view_types=front",
                    "front_frame",
                    self.front_image_pub,
                    self.front_info_pub,
                    self.front_camera_info,
                )
            elif self.image_mode == "rear":
                self._poll_single_image(
                    "/screenshot?view_types=rear",
                    "rear_frame",
                    self.rear_image_pub,
                    self.rear_info_pub,
                    self.rear_camera_info,
                )
            else:
                resp = self.http_get("/screenshot?view_types=front,rear")
                resp.raise_for_status()
                payload = resp.json()
                stamp = unix_to_stamp(self, payload.get("timestamp"))

                if "front_frame" in payload:
                    self.front_image_pub.publish(
                        self._to_ros_image(
                            decode_base64_image(payload["front_frame"]),
                            stamp,
                            self.front_camera_frame,
                        )
                    )
                    self.front_info_pub.publish(self._stamp_camera_info(self.front_camera_info, stamp))

                if "rear_frame" in payload:
                    self.rear_image_pub.publish(
                        self._to_ros_image(
                            decode_base64_image(payload["rear_frame"]),
                            stamp,
                            self.rear_camera_frame,
                        )
                    )
                    self.rear_info_pub.publish(self._stamp_camera_info(self.rear_camera_info, stamp))

        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image polling failed: {exc}")

    def _poll_single_image(self, path: str, key: str, image_pub, info_pub, camera_info: CameraInfo) -> None:
        resp = self.http_get(path)
        resp.raise_for_status()
        payload = resp.json()
        stamp = unix_to_stamp(self, payload.get("timestamp"))
        image_pub.publish(self._to_ros_image(decode_base64_image(payload[key]), stamp, camera_info.header.frame_id))
        info_pub.publish(self._stamp_camera_info(camera_info, stamp))

    def _to_ros_image(self, frame: np.ndarray, stamp: Time, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        return msg

    def _stamp_camera_info(self, msg: CameraInfo, stamp: Time) -> CameraInfo:
        stamped = CameraInfo()
        stamped.header.stamp = stamp
        stamped.header.frame_id = msg.header.frame_id
        stamped.height = msg.height
        stamped.width = msg.width
        stamped.distortion_model = msg.distortion_model
        stamped.d = list(msg.d)
        stamped.k = list(msg.k)
        stamped.r = list(msg.r)
        stamped.p = list(msg.p)
        return stamped

    # Data polling

    def poll_data(self) -> None:
        try:
            resp = self.http_get("/data")
            resp.raise_for_status()
            payload = resp.json()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"data polling failed: {exc}")
            return

        if payload is None:
            self.get_logger().warn("data polling returned empty payload")
            return

        sdk_ts = payload.get("timestamp")
        base_stamp = unix_to_stamp(self, sdk_ts)

        self._publish_scalar(self.battery_pub, UInt8, int(float(payload.get("battery", 0))))
        self._publish_scalar(self.signal_pub, UInt8, int(float(payload.get("signal_level", 0))))
        self._publish_scalar(self.gps_signal_pub, Float32, float(payload.get("gps_signal", 0.0)))
        self._publish_scalar(self.speed_pub, Float32, float(payload.get("speed", 0.0)))

        orientation = float(payload.get("orientation", 0.0))
        if not self.assume_orientation_is_degrees:
            orientation = math.degrees(orientation)
        self._publish_scalar(self.orientation_pub, Float32, orientation)

        self.publish_gps(payload, base_stamp)
        self.publish_imu_family(payload)
        self.publish_rpms(payload)

    def _publish_scalar(self, pub, msg_type, value: float) -> None:
        msg = msg_type()
        msg.data = value
        pub.publish(msg)

    # GPS

    def publish_gps(self, payload: Dict[str, Any], stamp: Time) -> None:
        if "latitude" not in payload or "longitude" not in payload:
            return

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.gps_frame
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(payload["latitude"])
        msg.longitude = float(payload["longitude"])
        msg.altitude = float(payload.get("altitude", 0.0))
        msg.position_covariance = [
            15.0,  0.0,  0.0,
             0.0, 15.0,  0.0,
             0.0,  0.0, 15.0
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(msg)

    # IMU / Magnetometer

    def publish_imu_family(self, payload: Dict[str, Any]) -> None:
        accels = payload.get("accels", [])
        gyros = payload.get("gyros", [])
        mags = payload.get("mags", [])

        # Publish Imu messages using accel/gyro streams.
        for acc, gyro in zip_longest_safe(accels, gyros):
            stamp = unix_to_stamp(self, pick_ts(acc, gyro, fallback=payload.get("timestamp")))
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.imu_frame

            # No orientation quaternion provided here.
            # robot_localization should use angular velocity and linear acceleration only.
            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity_covariance[0] = self.imu_angular_vel_cov[0]
            msg.angular_velocity_covariance[4] = self.imu_angular_vel_cov[1]
            msg.angular_velocity_covariance[8] = self.imu_angular_vel_cov[2]

            msg.linear_acceleration_covariance[0] = self.imu_linear_accel_cov[0]
            msg.linear_acceleration_covariance[4] = self.imu_linear_accel_cov[1]
            msg.linear_acceleration_covariance[8] = self.imu_linear_accel_cov[2]

            if acc is not None and len(acc) >= 3:
                if self.apply_imu_axis_remap:
                    ax_b, ay_b, az_b = self.remap_accel_to_body(acc)
                else:
                    ax_b = float(acc[0]) * self.accel_scale_to_mps2
                    ay_b = float(acc[1]) * self.accel_scale_to_mps2
                    az_b = float(acc[2]) * self.accel_scale_to_mps2

                msg.linear_acceleration.x = ax_b
                msg.linear_acceleration.y = ay_b
                msg.linear_acceleration.z = az_b

            if gyro is not None and len(gyro) >= 3: #bias tried removing it but adopted it in the localizatuion 
                if self.apply_imu_axis_remap:
                    gx_b, gy_b, gz_b = self.remap_gyro_to_body(gyro)
                else:
                    gx_b = (float(gyro[0])) * self.gyro_scale_to_rads
                    gy_b = (float(gyro[1])) * self.gyro_scale_to_rads
                    gz_b = (float(gyro[2])) * self.gyro_scale_to_rads

                msg.angular_velocity.x = gx_b
                msg.angular_velocity.y = gy_b
                msg.angular_velocity.z = gz_b

            self.imu_pub.publish(msg)

        # Publishing magnetometer separately.
        for sample in mags:
            if len(sample) < 3:
                continue

            stamp = unix_to_stamp(self, sample[3] if len(sample) > 3 else payload.get("timestamp"))
            msg = MagneticField()
            msg.header.stamp = stamp
            msg.header.frame_id = self.imu_frame

            if self.apply_imu_axis_remap:
                mx_b, my_b, mz_b = self.remap_mag_to_body(sample)
            else:
                mx_b = float(sample[0]) * self.magnetometer_scale_to_tesla
                my_b = float(sample[1]) * self.magnetometer_scale_to_tesla
                mz_b = float(sample[2]) * self.magnetometer_scale_to_tesla

            msg.magnetic_field.x = mx_b
            msg.magnetic_field.y = my_b
            msg.magnetic_field.z = mz_b

            msg.magnetic_field_covariance[0] = self.mag_cov[0]
            msg.magnetic_field_covariance[4] = self.mag_cov[1]
            msg.magnetic_field_covariance[8] = self.mag_cov[2]
            self.mag_pub.publish(msg)

    # Wheel RPMs
    def publish_rpms(self, payload: Dict[str, Any]) -> None:
        rpms = payload.get("rpms", [])
        if not rpms:
            return

        latest = rpms[-1]
        if len(latest) < 4:
            return

        msg = JointState()
        stamp = unix_to_stamp(self, latest[4] if len(latest) > 4 else payload.get("timestamp"))
        msg.header.stamp = stamp
        msg.name = list(self.rpm_joint_names)
        msg.velocity = [float(v) * self.rpm_to_rad_s for v in latest[:4]]

        if self.publish_joint_positions:
            if self.last_joint_stamp is None:
                self.last_joint_stamp = stamp
                self.last_joint_position = [0.0] * len(msg.velocity)
            else:
                dt = (stamp.sec - self.last_joint_stamp.sec) + (stamp.nanosec - self.last_joint_stamp.nanosec) * 1e-9
                if dt > 0.0:
                    self.last_joint_position = [p + v * dt for p, v in zip(self.last_joint_position, msg.velocity)]
                    self.last_joint_stamp = stamp
            msg.position = list(self.last_joint_position)

        self.joint_pub.publish(msg)

        if not self.publish_wheel_odom:
            return

        rpm_fl, rpm_fr, rpm_rl, rpm_rr = [float(v) for v in latest[:4]]
        omega_left  = (rpm_fl + rpm_rl) / 2.0 * self.rpm_to_rad_s  # rad/s
        omega_right = (rpm_fr + rpm_rr) / 2.0 * self.rpm_to_rad_s  # rad/s
        v_left  = omega_left  * self.wheel_radius_m   # m/s
        v_right = omega_right * self.wheel_radius_m   # m/s
        v_linear  = (v_right + v_left)  / 2.0         # forward m/s
        v_angular = (v_right - v_left)  / self.wheel_track_m  # rad/s

        unix_now = latest[4] if len(latest) > 4 else None
        if unix_now is not None and self._odom_last_stamp is not None:
            dt = unix_now - self._odom_last_stamp
            if 0.0 < dt < 1.0:  # sanity check
                self._odom_yaw += v_angular * dt
                self._odom_x   += v_linear * math.cos(self._odom_yaw) * dt
                self._odom_y   += v_linear * math.sin(self._odom_yaw) * dt
        if unix_now is not None:
            self._odom_last_stamp = unix_now

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        half_yaw = self._odom_yaw / 2.0
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular
        # Covariance: moderate uncertainty (RPMs are not encoder-accurate)
        odom.pose.covariance[0]  = 0.2   # x
        odom.pose.covariance[7]  = 0.2   # y
        odom.pose.covariance[35] = 0.05  # yaw
        odom.twist.covariance[0]  = 0.05  # vx
        odom.twist.covariance[35] = 0.05  # wz
        self.wheel_odom_pub.publish(odom)

    def on_cmd_vel(self, msg: Twist) -> None:
        linear = clamp(float(msg.linear.x), -1.0, 1.0)
        angular = clamp(float(msg.angular.z), -1.0, 1.0)
        self.last_cmd_vel_payload = {"command": {"linear": 0, "angular": 0, "lamp": 0}}
        self.last_cmd_vel_stamp = self.get_clock().now()

    def poll_control(self) -> None:
        if self.last_cmd_vel_stamp is None:
            return
            
        elapsed = (self.get_clock().now() - self.last_cmd_vel_stamp).nanoseconds / 1e9
        payload = self.last_cmd_vel_payload
        # Deadman switch: if no cmd_vel received for > 0.5s, stop
        if elapsed > 0.5:
            payload = {"command": {"linear": 0.0, "angular": 0.0, "lamp": 0}}

        try:
            resp = self.http_post("/control", payload)
            resp.raise_for_status()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"control send failed: {exc}", throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EarthRoverBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()