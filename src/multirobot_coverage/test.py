import numpy as np
import cv2
import yaml
import json

# ros modules
import rospy
import rospkg
from nav_msgs.msg import Path, MapMetaData, Odometry, OccupancyGrid
from geometry_msgs.msg import Pose

# custom modules
from robot import Robot
from covarage_plan import Voronoi
from path_plan import PathPlan
import param


## color code loading
pkg_path = rospkg.RosPack()
path = pkg_path.get_path("multirobot_coverage")
color_path = path + "/param/color_code.yaml"
print(color_path)
with open(color_path, "r") as file:
    color_code = yaml.safe_load(file)
# print("loaded color_code {}".format(color_code))

# parameters
ROBOT_NS = param.ROBOT_NS
DEFAULT_ODOM_TOPIC = param.DEFAULT_ODOM_TOPIC
DEFAULT_COSTMAP_TOPIC = param.DEFAULT_COSTMAP_TOPIC
EXHAUSTIVE_PLAN_TOPIC = param.EXHAUSTIVE_PLAN_TOPIC


def get_number_interested(find_str, to_be_searched_str):
    """return robot id
    Args:
        find_str = "an_" for agent
        to_be_searched_str (str) file directory
    Returns:
        robot index (int)
    """

    end_str = "_"
    if find_str in to_be_searched_str:
        start_idx = to_be_searched_str.find(find_str) + len(find_str)
        j = 0
        end_idx = start_idx
        while j < len(to_be_searched_str[start_idx:]):
            if to_be_searched_str[start_idx + j] == end_str:
                break
            j += 1

        end_idx = end_idx + j
        # print(end_idx)
        return to_be_searched_str[start_idx:end_idx]


def path_to_list(path_msg):
    path_list = []
    for pose_stamped in path_msg.poses:
        pose = pose_stamped.pose
        position = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
        orientation = {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w}
        path_list.append({"position": position, "orientation": orientation})
    return path_list


def save_path_to_file(path_list, file_name="path_data.json"):
    """
    Save the path list to a JSON file.

    :param path_list: List of dictionaries representing the path.
    :param file_name: Name of the file to save the data to.
    """
    with open(file_name, "w") as file:
        json.dump(path_list, file, indent=4)


def position_from_pose(pose):
    # type: (Pose) -> np.ndarray
    return position_from_point(pose.position)


def position_from_point(point):
    # type: (Point) -> np.ndarray
    return np.array((point.x, point.y))


class Planner:
    def __init__(self):
        self.total_agent = rospy.get_param("/total_agent")
        self.global_file_name = rospy.get_param("/global_file_name")
        self.team_member = [ROBOT_NS.format(member_idx) for member_idx in range(int(self.total_agent))]
        self.total_team_member = len(self.team_member)
        self.pose_saver = dict()
        self.path_saver = dict()
        self.path_save_bool = [False] * int(self.total_agent)
        self.rate = rospy.Rate(5)

        # just for a starting, static map subscribe. Not necessarily being used in a meaningful way
        self.map_service = "/static_map"
        print("Waiting for map service to come up...")
        rospy.wait_for_service(self.map_service)
        self.map_msg = rospy.wait_for_message("/robot_0/" + DEFAULT_COSTMAP_TOPIC, OccupancyGrid)  # costmap for navigation

        self.resize = (self.map_msg.info.width, self.map_msg.info.height)

        # agent pose subscribe
        for i in range(self.total_team_member):
            ######### teammate pose subscription
            s = "def teammate_pose_" + self.team_member[i] + "(self, msg): \n"
            s += "\t _position = position_from_pose(msg.pose.pose) \n"
            s += "\t self.pose_saver[{}] = _position \n".format(i)
            # s += "\t print('working for team {}') \n".format(i)
            exec(s)
            exec("setattr(Planner, 'callback_teammate_pose_" + self.team_member[i] + "', teammate_pose_" + self.team_member[i] + ")")

            exec(
                "msg = rospy.wait_for_message('/"
                + str(self.team_member[i])
                + "/{}', Odometry) \n".format(DEFAULT_ODOM_TOPIC)
                + "self.callback_teammate_pose_"
                + self.team_member[i]
                + "(msg)"
            )
            exec(
                "rospy.Subscriber('/"
                + str(self.team_member[i])
                + "/{}', Odometry, self.callback_teammate_pose_".format(DEFAULT_ODOM_TOPIC)
                + self.team_member[i]
                + ", queue_size = 1)"
            )

            exec(
                "self."
                + str(self.team_member[i])
                + "_pub = rospy.Publisher('/"
                + str(self.team_member[i])
                + "/{}', Path, queue_size=1)".format(EXHAUSTIVE_PLAN_TOPIC)
            )

    def decompose_and_plan(self):
        """
        1. decompose area depending on the robot locations
        2. plan a lawnmower path withinn the allocated area
        """

        if len(self.pose_saver) > 0:
            # ------------------------------------------------------------
            ##### divide region #####
            robots = dict()
            for i in range(self.total_team_member):
                color = color_code[ROBOT_NS.format(i)]["color_code"]
                position_i = self.pose_saver.get(i)
                if position_i is not None:

                    pose_x, pose_y = position_i[0], position_i[1]
                    pose_i = Pose()
                    pose_i.position.x = pose_x
                    pose_i.position.y = pose_y
                    pose_i.position.z = 0.0

                    robot_i = Robot(i, pose_i, 1.0, color)

                    robots[i] = robot_i
                    print(pose_i)

            # print("robots {}".format(robots))

            voronoi = Voronoi(self.map_service, robots)
            for i in range(30):
                occ_grid, tesselation_image = voronoi.tesselation()

            # ------------------------------------------------------------
            ##### path plan #####
            occ_grid = cv2.resize(occ_grid, self.resize)
            tesselation_image = cv2.resize(tesselation_image, self.resize)

            map_info = MapMetaData()
            map_info = voronoi.graph.map_info

            env_name = ""
            if "city" in self.global_file_name:
                env_name = "city"
            elif "empty" in self.global_file_name:
                env_name = "empty"
            elif "shanghai" in self.global_file_name:
                env_name = "shanghai"
            env_no = get_number_interested("env_", self.global_file_name)
            agent_no = get_number_interested("an_", self.global_file_name)

            for i in range(self.total_team_member):
                robot_i = robots.get(i)
                path_plan_i = PathPlan(occ_grid, map_info, tesselation_image, robot_i)
                path_i = path_plan_i.plan()

                self.path_saver[i] = path_i

                if self.path_save_bool[i] is False:
                    path_list = path_to_list(path_i)
                    save_path_to_file(path_list, path + "/path/{}_{}_{}_{}_path.json".format(env_name, agent_no, env_no, i))
                    self.path_save_bool[i] = True
            # ------------------------------------------------------------

    def spin(self):
        while not rospy.is_shutdown():
            for i in range(self.total_team_member):
                path_i = self.path_saver.get(i)
                if path_i is not None:
                    robot_i_pub = eval("self." + str(self.team_member[i]) + "_pub")
                    robot_i_pub.publish(path_i)
                    self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("planner")

    planner = Planner()
    planner.decompose_and_plan()

    planner.spin()
