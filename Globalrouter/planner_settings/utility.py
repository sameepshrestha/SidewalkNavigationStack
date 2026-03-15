import math
from typing import List, Tuple

import polyline


def haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Distance in metres between two (lat, lon) points."""
    R = 6_371_000
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def decode_route_shape(encoded_shape: str) -> List[Tuple[float, float]]:
    """Decode Valhalla's encoded polyline (6-digit precision) to (lat, lon) pairs."""
    return polyline.decode(encoded_shape, precision=6)
