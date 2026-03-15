import sys, os
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'global_planner'))

import folium
from route_planner import RoutePlanner
from utility import haversine


START_GPS = (38.837858, -77.294312)
GOAL_GPS  = (38.837899, -77.295573)


def main():
    planner = RoutePlanner()
    waypoints = planner.generate_route(
        start=[START_GPS[1], START_GPS[0]],
        goals=[[GOAL_GPS[1], GOAL_GPS[0]]]
    )

    print(f"Route: {len(planner.route_coords)} shape points from Valhalla")

    print("\n── Maneuvers ──")
    for man in planner.route_maneuvers:
        print(f"  {man['instruction']}  ({man['length']*1000:.0f}m)")

    total_dist = sum(
        haversine(planner.route_coords[i], planner.route_coords[i+1])
        for i in range(len(planner.route_coords) - 1)
    )
    print(f"\nTotal distance: {total_dist:.1f} m")
    print(f"Waypoints ({planner.config.waypoint_spacing_m}m): {len(waypoints)}")

    # Folium map
    center = ((START_GPS[0] + GOAL_GPS[0]) / 2,
              (START_GPS[1] + GOAL_GPS[1]) / 2)
    m = folium.Map(location=center, zoom_start=17)

    folium.Marker(START_GPS, popup="START",
                  icon=folium.Icon(color="green", icon="play")).add_to(m)
    folium.Marker(GOAL_GPS, popup="GOAL",
                  icon=folium.Icon(color="red", icon="flag")).add_to(m)

    folium.PolyLine(planner.route_coords, color="blue",
                    weight=3, opacity=0.6).add_to(m)

    for i, wp in enumerate(waypoints):
        folium.CircleMarker(wp, radius=3, color="orange",
                            fill=True, fill_opacity=0.8).add_to(m)

    m.save("route_test_map.html")
    print(f"\nMap saved to: route_test_map.html")


if __name__ == "__main__":
    main()
