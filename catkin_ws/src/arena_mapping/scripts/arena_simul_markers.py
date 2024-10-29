#!/usr/bin/env python3
import rospy
import tf2_ros
# import tf_conversions
import geometry_msgs.msg
from visualization_msgs.msg import Marker

def publish_light_bulb():
    rospy.init_node('arena_simul_markers')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    light_bulb = Marker()
    light_bulb.header.frame_id = "map"
    light_bulb.header.stamp = rospy.Time.now()
    light_bulb.type = Marker.SPHERE
    light_bulb.action = Marker.ADD

    light_bulb.pose.position.x = 2.30
    light_bulb.pose.position.y = 0.50
    light_bulb.pose.position.z = 0.10
    light_bulb.pose.orientation.x = 0.0
    light_bulb.pose.orientation.y = 0.0
    light_bulb.pose.orientation.z = 0.0
    light_bulb.pose.orientation.w = 1.0

    light_bulb.scale.x = 0.1
    light_bulb.scale.y = 0.1
    light_bulb.scale.z = 0.1

    light_bulb.color.r = 1.0
    light_bulb.color.g = 1.0
    light_bulb.color.b = 1.0
    light_bulb.color.a = 1.0

    rate = rospy.Rate(10) 

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        pub.publish(light_bulb)

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "light_bulb"
        t.transform.translation.x = 2.30
        t.transform.translation.y = 0.50
        t.transform.translation.z = 0.10

        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)
        # br.sendTransform((light_bulb.pose.position.x, light_bulb.pose.position.y, light_bulb.pose.position.z), (0, 0, 0, 1), rospy.Time.now(), "light_bulb","map")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_light_bulb()
    except rospy.ROSInterruptException:
        pass
