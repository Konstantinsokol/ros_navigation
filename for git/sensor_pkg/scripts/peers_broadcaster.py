#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from robot_msgs.msg import peers

""" нода подписывается на /peers : robot_msgs/peers в. front_tag_ID храниться ID 
метки. в зависимость от этого нода перенаправляет данные в /front_tag и /back_tag перобразовывая к дуругому типу сообщения(geometry_msgs/PoseWithCovarianceStamped)"""

class Node():
    rospy.init_node('peers_broadcaster')
    front_tag_ID = str(rospy.get_param('~front_tag_ID'))
    back_tag_ID = str(rospy.get_param('~back_tag_ID'))
    #front_tag_ID = "00b4"
    #back_tag_ID = "c30c"
    r = rospy.Rate(60)
    _msg = PoseWithCovarianceStamped()

    def __init__(self) -> None:
        rospy.Subscriber('/peers', PointStamped, self.handlerTagPose)
        self.front_pub_pose = rospy.Publisher(
            '/front_tag', PoseWithCovarianceStamped, queue_size=10)
        self.back_pub_pose = rospy.Publisher(
            '/back_tag', PoseWithCovarianceStamped, queue_size=10)
        rospy.spin()

    def handlerTagPose(self, data):
        if data.header.frame_id == self.front_tag_ID:
            self._msg.header.frame_id = "map"
            msg = self.composeMessage(data.point.x, data.point.y)
            self.front_pub_pose.publish(msg)
            self.r.sleep()
        elif data.header.frame_id == self.back_tag_ID:
            self._msg.header.frame_id = "back_tag_link"
            msg = self.composeMessage(data.point.x, data.point.y)
            self.back_pub_pose.publish(msg)
            self.r.sleep()

    def composeMessage(self, x, y):
        msg = self._msg
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0
        msg.pose.covariance = [0.3, 0, 0, 0, 0, 0,
                               0, 0.3, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0]
        return msg

if __name__== "__main__":
    Node()