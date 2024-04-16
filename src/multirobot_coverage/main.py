import cv2
import rospy
from nav_msgs.msg import Path, MapMetaData
from geometry_msgs.msg import Pose
from robot import Robot
from covarage_plan import Voronoi
from path_plan import PathPlan


if __name__ == "__main__":
    rospy.init_node("main")
    map_service = "/static_map"
    print("Waiting for map service to come up...")
    rospy.wait_for_service(map_service)
    print("Map service is active")

    pub1 = rospy.Publisher("/robot_1/planned_path", Path, queue_size=1, latch=True)
    pub2 = rospy.Publisher("/robot_2/planned_path", Path, queue_size=1, latch=True)

    r1_pose = Pose()
    r1_pose.position.x = 42
    r1_pose.position.y = 24
    r1_pose.position.z = 0.0

    r2_pose = Pose()
    r2_pose.position.x = 42
    r2_pose.position.y = 25
    r2_pose.position.z = 0.0

    # r1_pose = Pose()
    # r1_pose.position.x = 18
    # r1_pose.position.y = 22
    # r1_pose.position.z = 0.0

    # r2_pose = Pose()
    # r2_pose.position.x = -13
    # r2_pose.position.y = -27
    # r2_pose.position.z = 0.0

    robot1 = Robot(1, r1_pose, 1.0, [0, 255, 0])
    robot2 = Robot(2, r2_pose, 1.0, [0, 0, 255])
    robots = {robot1.robot_id: robot1, robot2.robot_id: robot2}
    voronoi = Voronoi(map_service, robots)
    for i in range(30):
        occ_grid, tesselation_image = voronoi.tesselation()

    occ_grid = cv2.resize(occ_grid, (2571, 747))
    tesselation_image = cv2.resize(tesselation_image, (2571, 747))

    # occ_grid = cv2.resize(occ_grid, (325, 325))
    # tesselation_image = cv2.resize(tesselation_image, (325, 325))
    """ for i in range(tesselation_image.shape[0]):
    s = ""
    for j in range(tesselation_image.shape[1]):
      s += str(tesselation_image[i,j])
    print(s) """
    map_info = MapMetaData()
    map_info = voronoi.graph.map_info
    print("Width: ", map_info.width)
    print("Height: ", map_info.height)
    print("Res: ", map_info.resolution)
    """ map_info.height = voronoi.graph.height
    map_info.width = voronoi.graph.width
    map_info.origin = voronoi.graph.origin
    map_info.resolution = voronoi.graph.resolution """
    # robot2 = Robot(2, r2_pose, 1.0, 100)

    path_plan1 = PathPlan(occ_grid, map_info, tesselation_image, robot1)
    path_plan2 = PathPlan(occ_grid, map_info, tesselation_image, robot2)
    path1 = path_plan1.plan()
    path2 = path_plan2.plan()
    pub1.publish(path1)
    pub2.publish(path2)

    rospy.spin()
