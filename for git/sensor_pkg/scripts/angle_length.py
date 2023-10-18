#!/usr/bin/python3
import rospy
from robot_msgs.msg import peers
from setting_pkg.const import *
import math
from dataclasses import dataclass
from geometry_msgs.msg import TwistWithCovarianceStamped, PointStamped

@dataclass
class tag():
    id: str
    x: float = None
    y: float = None

fronttag = tag("00b4")
backtag = tag("c30c")

class angl_len_server():

    rospy.init_node(ROBO_PATHPLANING_NODE_NAME)
    r = rospy.Rate(60)
    count = 0
    def __init__(self) -> None:
        rospy.loginfo(
            f"{rospy.get_name()} started")
        rospy.Subscriber('/peers', PointStamped, self.handel_peers)
        self.yaw_peers = rospy.Publisher('/corner_from_peers_link', TwistWithCovarianceStamped, queue_size=10)
        rospy.spin()

    def Response(self) -> None:

        rospy.loginfo(f"{rospy.get_name()}: fronttag: {fronttag.x}, {fronttag.y}; backtag: {backtag.x}, {backtag.y};")
        # Координаты точек
        front_x = round(fronttag.x, 2)
        front_y = round(fronttag.y ,2)

        back_x = round(backtag.x, 2)
        back_y = round(backtag.y ,2)

        if front_x == back_x and front_y > back_y:
            angle = 90
        if front_x == back_x and front_y < back_y:
            angle = -90
        elif front_y == back_y and front_x > back_x:
            angle = 180
        else:
            angle = math.atan2(front_y - back_y, front_x - back_x)
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.header.seq = self.count
        msg.twist.twist.angular.z = angle
        self.yaw_peers.publish(msg)
        self.count += 1
        self.r.sleep()

    def handel_peers(self,data) -> None:
        rospy.loginfo(
            f" ID:{data.header.frame_id}, x:{data.point.x}, y:{data.point.y}")
        if data.header.frame_id == fronttag.id:
            fronttag.x, fronttag.y = data.point.x, data.point.y
        elif data.header.frame_id == backtag.id:
            backtag.x, backtag.y = data.point.x, data.point.y
        self.Response()

if __name__ == "__main__":
    angl_len_server()
