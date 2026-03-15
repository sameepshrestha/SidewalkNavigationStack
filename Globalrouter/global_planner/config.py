from dataclasses import dataclass


@dataclass
class PlannerConfig:
    valhalla_url: str = "https://valhalla1.openstreetmap.de/route"

    walkway_factor: float = 0.5
    alley_factor: float = 5.0
    driveway_factor: float = 5.0
    sidewalk_factor: float = 0.5
    exclude_steps: bool = True

    waypoint_spacing_m: float = 13.0

    @property
    def costing_model(self) -> str:
        return "pedestrian"

    def to_costing_options(self) -> dict:
        opts = {
            "walkway_factor": self.walkway_factor,
            "alley_factor": self.alley_factor,
            "driveway_factor": self.driveway_factor,
            "sidewalk_factor": self.sidewalk_factor,
        }
        if self.exclude_steps:
            opts["step_penalty"] = 100000
        return {"pedestrian": opts}
