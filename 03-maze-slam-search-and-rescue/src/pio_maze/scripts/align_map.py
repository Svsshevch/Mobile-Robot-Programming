#!/usr/bin/env python3
"""
align_map.py - shift a saved SLAM map so its frame coincides with the Gazebo
world frame.

slam_toolbox initialises the map frame at the robot's spawn pose, so a freshly
saved map is expressed relative to spawn (robot at 0,0).  The robot spawns at
world (-7.0, -6.5); adding that offset to the map's origin re-expresses the map
in world coordinates, so the assignment's world-frame victim goals and the AMCL
initial pose work directly during Part 2.

Usage:
    ros2 run pio_maze align_map.py <map.yaml> [off_x=-7.0] [off_y=-6.5]
"""
import sys
import yaml


def main():
    if len(sys.argv) < 2:
        print("usage: align_map.py <map.yaml> [off_x] [off_y]")
        sys.exit(1)
    path = sys.argv[1]
    ox = float(sys.argv[2]) if len(sys.argv) > 2 else -7.0
    oy = float(sys.argv[3]) if len(sys.argv) > 3 else -6.5
    with open(path) as f:
        m = yaml.safe_load(f)
    m['origin'][0] += ox
    m['origin'][1] += oy
    with open(path, 'w') as f:
        yaml.safe_dump(m, f, default_flow_style=False, sort_keys=False)
    print(f"aligned '{path}' origin -> {m['origin']}")


if __name__ == '__main__':
    main()
