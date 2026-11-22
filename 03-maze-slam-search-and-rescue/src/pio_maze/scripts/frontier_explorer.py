#!/usr/bin/env python3
"""
frontier_explorer.py - autonomous frontier exploration driving Nav2.

Repeatedly:
  1. read the slam_toolbox /map,
  2. find frontiers (free cells touching unknown), cluster them,
  3. pick the best cluster (large + near), snap goal to free space,
  4. send it to Nav2 (navigate_to_pose),
  5. when no frontiers remain -> save the map and return to start.

Because frontiers lie on the boundary between known-free and unknown space, the
robot is driven into open areas and side-passages until the map is complete.
"""
import math
import os
import subprocess

import yaml
from collections import deque
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import tf2_ros

FREE_MAX = 25
OCC_MIN = 65
UNKNOWN = -1


class S(Enum):
    WAIT_MAP = 0
    PICK = 1
    NAV = 2
    DONE = 3


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('start_x', 0.0)     # slam-frame start (spawn = 0,0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('min_cluster', 5)    # ignore 1-4 cell noise only
        self.declare_parameter('min_goal_dist', 0.4)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('nav_timeout', 40.0)
        self.declare_parameter('save_map', True)
        self.declare_parameter('map_name', 'generated_map')
        self.declare_parameter('map_dir', '')        # '' -> package share maps/
        self.declare_parameter('align', True)        # shift map into world frame
        self.declare_parameter('spawn_x', -7.0)      # world spawn (alignment offset)
        self.declare_parameter('spawn_y', -6.5)
        self.declare_parameter('return_home', True)
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.min_cluster = self.get_parameter('min_cluster').value
        self.min_goal_dist = self.get_parameter('min_goal_dist').value
        self.bl_radius = self.get_parameter('blacklist_radius').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.do_save = self.get_parameter('save_map').value
        self.map_name = self.get_parameter('map_name').value
        self.map_dir = self.get_parameter('map_dir').value
        self.do_align = self.get_parameter('align').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.do_home = self.get_parameter('return_home').value

        self.state = S.WAIT_MAP
        self.map = None
        self.blacklist = []
        self.goal_handle = None
        self.goal_t0 = None
        self.empty_streak = 0
        self.going_home = False

        mqos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, mqos)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(2.0, self.tick)
        # periodic checkpoint of the map during exploration
        self.create_timer(60.0, self.periodic_save)
        self.get_logger().info('frontier_explorer: waiting for map...')

    def periodic_save(self):
        if self.do_save and self.map is not None and self.state in (S.PICK, S.NAV):
            self.save_map()

    def map_cb(self, m):
        self.map = m
        if self.state == S.WAIT_MAP:
            self.state = S.PICK
            self.get_logger().info('first map received.')

    def robot_xy(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint',
                                             rclpy.time.Time(),
                                             timeout=Duration(seconds=1.0))
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'TF map->base_footprint not ready: {e}')
            return None

    # ---------------- frontier detection ----------------
    def find_frontiers(self):
        g = self.map.data
        w, h = self.map.info.width, self.map.info.height
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        def free(x, y):
            return 0 <= g[y * w + x] <= FREE_MAX

        def unknown(x, y):
            return g[y * w + x] == UNKNOWN

        fcells = set()
        for y in range(1, h - 1):
            row = y * w
            for x in range(1, w - 1):
                v = g[row + x]
                if not (0 <= v <= FREE_MAX):
                    continue
                if (unknown(x + 1, y) or unknown(x - 1, y) or
                        unknown(x, y + 1) or unknown(x, y - 1)):
                    fcells.add((x, y))

        clusters = []
        seen = set()
        for c in fcells:
            if c in seen:
                continue
            q = deque([c]); seen.add(c); cl = []
            while q:
                cx, cy = q.popleft(); cl.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in fcells and n not in seen:
                            seen.add(n); q.append(n)
            if len(cl) >= self.min_cluster:
                clusters.append(cl)

        robot = self.robot_xy()
        if robot is None:
            return None
        rx, ry = robot
        rx_cell = int((rx - ox) / res)
        ry_cell = int((ry - oy) / res)
        out = []
        for cl in clusters:
            cell = self.safe_goal_cell(cl, rx_cell, ry_cell)
            if cell is None:        # no reachable in-bounds free cell
                continue
            cx, cy = cell
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            if any(math.hypot(wx - bx, wy - by) < self.bl_radius
                   for bx, by in self.blacklist):
                continue
            d = math.hypot(wx - rx, wy - ry)
            if d < self.min_goal_dist:
                continue
            out.append({'size': len(cl), 'xy': (wx, wy), 'd': d,
                        'score': len(cl) / (d + 1.0)})
        out.sort(key=lambda f: f['score'], reverse=True)
        return out

    def safe_goal_cell(self, cluster, rx_cell, ry_cell, margin=4):
        """Return an in-bounds FREE cell for this frontier cluster.

        The cluster centroid sits on the known/unknown edge, which can fall on
        the costmap border.  Walk from the centroid toward the robot and return
        the first cell that is free and at least `margin` cells inside the map,
        so Nav2 can plan to it.
        """
        g = self.map.data
        w, h = self.map.info.width, self.map.info.height
        mx = sum(c[0] for c in cluster) / len(cluster)
        my = sum(c[1] for c in cluster) / len(cluster)
        steps = max(1, int(math.hypot(rx_cell - mx, ry_cell - my)))
        for i in range(steps + 1):
            t = i / steps
            cx = int(round(mx + (rx_cell - mx) * t))
            cy = int(round(my + (ry_cell - my) * t))
            if margin <= cx < w - margin and margin <= cy < h - margin:
                if 0 <= g[cy * w + cx] <= FREE_MAX:
                    return (cx, cy)
        return None

    # ---------------- state machine ----------------
    def tick(self):
        if self.state == S.PICK:
            self.pick()
        elif self.state == S.NAV:
            if self.goal_t0 is not None:
                el = (self.get_clock().now() - self.goal_t0).nanoseconds / 1e9
                if el > self.nav_timeout and self.goal_handle is not None:
                    self.get_logger().warn('goal timeout -> cancel')
                    self.goal_handle.cancel_goal_async()

    def pick(self):
        if self.map is None:
            return
        fr = self.find_frontiers()
        if fr is None:
            return  # pose not ready
        if not fr:
            self.empty_streak += 1
            if self.empty_streak < 20:
                self.get_logger().info(
                    f'no frontiers ({self.empty_streak}/5), retrying...')
                return
            self.finish()
            return
        self.empty_streak = 0
        best = fr[0]
        wx, wy = best['xy']
        self.get_logger().info(
            f"goal -> ({wx:.2f},{wy:.2f}) size={best['size']} d={best['d']:.2f} "
            f"({len(fr)} frontiers)")
        self.send(wx, wy, self.explore_done)
        self.state = S.NAV

    def send(self, x, y, cb):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose server unavailable')
            return
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.w = 1.0
        self.goal_t0 = self.get_clock().now()
        self._pending = (x, y)
        f = self.nav.send_goal_async(g)
        f.add_done_callback(lambda fut: self.goal_resp(fut, cb))

    def goal_resp(self, fut, cb):
        h = fut.result()
        if not h.accepted:
            self.get_logger().warn('goal rejected')
            self.blacklist.append(self._pending)
            self.state = S.PICK
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(cb)

    def explore_done(self, fut):
        st = fut.result().status
        self.goal_handle = None
        self.goal_t0 = None
        if st != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'nav status {st}; blacklist goal')
            self.blacklist.append(self._pending)
        self.state = S.PICK

    def finish(self):
        self.get_logger().info('Exploration complete.')
        self.state = S.DONE
        if self.do_save:
            self.save_map()
        if self.do_home:
            self.get_logger().info(
                f'returning home ({self.start_x},{self.start_y})')
            self.send(self.start_x, self.start_y, self.home_done)
        else:
            self.timer.cancel()

    def home_done(self, fut):
        st = fut.result().status
        if st == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Back at start. Mapping complete.')
        else:
            self.get_logger().warn(f'return-home status {st}')
        self.timer.cancel()

    def save_map(self):
        # default save location: the package's maps/ folder (share dir)
        map_dir = self.map_dir
        if not map_dir:
            try:
                from ament_index_python.packages import get_package_share_directory
                map_dir = os.path.join(
                    get_package_share_directory('pio_maze'), 'maps')
            except Exception:
                map_dir = os.getcwd()
        os.makedirs(map_dir, exist_ok=True)
        path = os.path.join(map_dir, self.map_name)
        self.get_logger().info(f'saving map -> {path}')
        try:
            r = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', path, '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', 'save_map_timeout:=12.0'],
                capture_output=True, text=True, timeout=40)
            if r.returncode != 0:
                self.get_logger().error(f'save failed: {r.stderr}')
                return
            self.get_logger().info('map saved')
        except Exception as e:
            self.get_logger().error(f'save error: {e}')
            return
        # auto-align the saved map into the world frame (shift origin by spawn)
        if self.do_align:
            try:
                yaml_path = path + '.yaml'
                with open(yaml_path) as f:
                    m = yaml.safe_load(f)
                m['origin'][0] += self.spawn_x
                m['origin'][1] += self.spawn_y
                with open(yaml_path, 'w') as f:
                    yaml.safe_dump(m, f, default_flow_style=False, sort_keys=False)
                self.get_logger().info(
                    f'map aligned to world frame, origin -> {m["origin"]}')
            except Exception as e:
                self.get_logger().error(f'align error: {e}')


def main():
    rclpy.init()
    n = FrontierExplorer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
