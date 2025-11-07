#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion


class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr, min_x, min_y, max_x, max_y):
        self.resolution = resolution
        self.rr = rr
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.obstacle_map = None
        self.x_width = 0
        self.y_width = 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                rospy.logwarn("Open set empty - no path")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                rospy.loginfo("Goal found!")
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)
                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        rx.reverse()
        ry.reverse()
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx = [self.calc_grid_position(goal_node.x, self.min_x)]
        ry = [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        ix = int(round((px - self.min_x) / self.resolution))
        iy = int(round((py - self.min_y) / self.resolution))
        if ix < 0 or iy < 0 or ix >= self.x_width or iy >= self.y_width:
            return False
        if self.obstacle_map[ix][iy]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.x_width = int(round((self.max_x - self.min_x) / self.resolution)) + 1
        self.y_width = int(round((self.max_y - self.min_y) / self.resolution)) + 1
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    if math.hypot(iox - x, ioy - y) <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        return [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]


class AStarNode:
    def __init__(self):
        rospy.init_node('astar_planner_node_pose_stamped')

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.path_pub = rospy.Publisher('/astar_path', Path, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.map_msg = None
        self.robot_pose = None
        self.goal = None
        self.path = []

        self.rate = rospy.Rate(10)
        self.robot_radius = 0.35
        self.kp_lin = 0.8
        self.kp_ang = 2.0
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0
        self.follow_tolerance = 0.2
        self.occupancy_threshold = 50

        rospy.loginfo("A* planner (using /robot_pose from get_pose.py) started.")

    def pose_callback(self, msg):
        # Convertir quaternion -> yaw
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def map_callback(self, msg):
        self.map_msg = msg

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Goal received: {self.goal}")
        self.plan_path()

    def plan_path(self):
        if self.map_msg is None or self.robot_pose is None:
            rospy.logwarn("Waiting for map and robot pose...")
            return

        msg = self.map_msg
        res = msg.info.resolution
        ox, oy = [], []
        for iy in range(msg.info.height):
            for ix in range(msg.info.width):
                i = ix + iy * msg.info.width
                if msg.data[i] > self.occupancy_threshold:
                    x = msg.info.origin.position.x + ix * res
                    y = msg.info.origin.position.y + iy * res
                    ox.append(x)
                    oy.append(y)

        min_x = msg.info.origin.position.x
        min_y = msg.info.origin.position.y
        max_x = min_x + msg.info.width * res
        max_y = min_y + msg.info.height * res

        planner = AStarPlanner(ox, oy, res, self.robot_radius, min_x, min_y, max_x, max_y)

        sx, sy, _ = self.robot_pose
        gx, gy = self.goal
        rx, ry = planner.planning(sx, sy, gx, gy)

        if not rx:
            rospy.logwarn("A* failed to find a path.")
            return

        self.path = list(zip(rx, ry))
        self.publish_path()
        rospy.loginfo(f"Path planned ({len(rx)} points).")
        self.follow_path()

    def publish_path(self):
        path_msg = Path()
        path_msg.header = Header(frame_id="map", stamp=rospy.Time.now())
        for (x, y) in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = Point(x, y, 0)
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def follow_path(self):
        idx = 0
        while idx < len(self.path) and not rospy.is_shutdown():
            if self.robot_pose is None:
                self.rate.sleep()
                continue

            rx, ry, ryaw = self.robot_pose
            tx, ty = self.path[idx]
            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            ang_err = self.angle_diff(angle_to_target, ryaw)

            if dist < self.follow_tolerance:
                idx += 1
                continue

            cmd = Twist()
            if abs(ang_err) > 0.4:
                cmd.linear.x = 0
                cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, self.kp_ang * ang_err))
            else:
                cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, self.kp_lin * dist))
                cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, self.kp_ang * ang_err))

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.loginfo("Path following complete.")

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


if __name__ == '__main__':
    try:
        node = AStarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
