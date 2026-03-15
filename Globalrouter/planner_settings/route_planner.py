import json
import requests
from typing import List, Tuple

import sys, os
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'global_planner'))

from utility import haversine, decode_route_shape
from config import PlannerConfig


class RoutePlanner:
    def __init__(self, config: PlannerConfig = None):
        self.config = config or PlannerConfig()
        self.route_coords = []
        self._static_waypoints = []
        self.maneuvers = []

    def generate_route(
        self,
        start: List[float],
        goals: List[List[float]]
    ) -> List[Tuple[float, float]]:
        """
        Generate a pedestrian route from start through all goals in order.

        Args:
            start: [longitude, latitude]
            goals: [[lon, lat], ...] ordered goal points

        Returns:
            List of dense (lat, lon) static waypoints.
        """
        locations = [{"lon": start[0], "lat": start[1]}]
        for g in goals:
            locations.append({"lon": g[0], "lat": g[1]})

        request = {
            "locations": locations,
            "costing": self.config.costing_model,
            "costing_options": self.config.to_costing_options(),
        }

        resp = requests.post(self.config.valhalla_url, json=request)
        if resp.status_code != 200:
            raise RuntimeError(f"Valhalla API error {resp.status_code}: {resp.text}")
        response = resp.json()

        self.route_coords = []
        for leg in response["trip"]["legs"]:
            decoded = decode_route_shape(leg["shape"])
            if self.route_coords and decoded:
                decoded = decoded[1:]
            self.route_coords.extend(decoded)

        self.maneuvers = []
        for leg in response["trip"]["legs"]:
            self.maneuvers.extend(leg.get("maneuvers", []))

        self._static_waypoints = self._interpolate_waypoints(
            self.route_coords,
            self.config.waypoint_spacing_m
        )
        return self._static_waypoints

    def _interpolate_waypoints(
        self,
        coords: List[Tuple[float, float]],
        spacing_m: float
    ) -> List[Tuple[float, float]]:
        """
        Dense waypoints at fixed spacing. All original route points
        (turns, intersections) are preserved.
        """
        if len(coords) < 2:
            return list(coords)

        waypoints = [coords[0]]

        for i in range(1, len(coords)):
            p_prev = coords[i - 1]
            p_curr = coords[i]
            segment_dist = haversine(p_prev, p_curr)

            if segment_dist <= spacing_m:
                waypoints.append(p_curr)
            else:
                n_points = int(segment_dist // spacing_m)
                for j in range(1, n_points + 1):
                    ratio = j * spacing_m / segment_dist
                    lat = p_prev[0] + ratio * (p_curr[0] - p_prev[0])
                    lon = p_prev[1] + ratio * (p_curr[1] - p_prev[1])
                    waypoints.append((lat, lon))

                if waypoints[-1] != p_curr:
                    waypoints.append(p_curr)

        return waypoints

    @property
    def static_waypoints(self) -> List[Tuple[float, float]]:
        return self._static_waypoints

    @property
    def route_maneuvers(self) -> list:
        return self.maneuvers