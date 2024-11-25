#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


light_bulbs = [False, False]

def getParameters():
    global light_bulbs
    light_bulbs = rospy.get_param("/hardware/light_bulbs")


def publish_light_bulb():
    rospy.init_node('arena_simul_markers')
    topic = 'visualization_marker_array'
    pub_markers = rospy.Publisher(topic, MarkerArray, queue_size=1000)

    markerArray = MarkerArray()

    rospy.set_param("/hardware/light_bulbs", light_bulbs)

    
    light_bulb_1 = Marker()
    light_bulb_1.header.frame_id = "map"
    light_bulb_1.header.stamp = rospy.Time.now()
    light_bulb_1.type = Marker.SPHERE
    light_bulb_1.action = Marker.ADD

    light_bulb_1.pose.position.x = 0.30
    light_bulb_1.pose.position.y = 0.50
    light_bulb_1.pose.position.z = 0.10
    light_bulb_1.pose.orientation.x = 0.0
    light_bulb_1.pose.orientation.y = 0.0
    light_bulb_1.pose.orientation.z = 0.0
    light_bulb_1.pose.orientation.w = 1.0

    light_bulb_1.scale.x = 0.1
    light_bulb_1.scale.y = 0.1
    light_bulb_1.scale.z = 0.1

    light_bulb_1.color.r = 1.0
    light_bulb_1.color.g = 1.0
    light_bulb_1.color.b = 1.0
    light_bulb_1.color.a = 1.0

    light_bulb_2 = Marker()
    light_bulb_2.header.frame_id = "map"
    light_bulb_2.header.stamp = rospy.Time.now()
    light_bulb_2.type = Marker.SPHERE
    light_bulb_2.action = Marker.ADD
    light_bulb_2.pose.position.x = 2.30
    light_bulb_2.pose.position.y = 0.50
    light_bulb_2.pose.position.z = 0.10
    light_bulb_2.pose.orientation.x = 0.0
    light_bulb_2.pose.orientation.y = 0.0
    light_bulb_2.pose.orientation.z = 0.0
    light_bulb_2.pose.orientation.w = 1.0
    light_bulb_2.scale.x = 0.1
    light_bulb_2.scale.y = 0.1
    light_bulb_2.scale.z = 0.1
    light_bulb_2.color.r = 1.0
    light_bulb_2.color.g = 1.0
    light_bulb_2.color.b = 1.0
    light_bulb_2.color.a = 1.0

    rate = rospy.Rate(10) 

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()

    # markerArray.markers.pop(0)
    markerArray.markers.append(light_bulb_1)
    markerArray.markers.append(light_bulb_2)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1



    while not rospy.is_shutdown():
        getParameters()

        if light_bulbs[0]:
            light_bulb_1.color.b = 0.0
        else:
            light_bulb_1.color.b = 1.0

        if light_bulbs[1]:
            light_bulb_2.color.b = 0.0
        else:
            light_bulb_2.color.b = 1.0

        pub_markers.publish(markerArray)

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "light_bulb_1"
        t.transform.translation.x = 0.30
        t.transform.translation.y = 0.50
        t.transform.translation.z = 0.10

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "light_bulb_2"
        t.transform.translation.x = 2.30
        t.transform.translation.y = 0.50
        t.transform.translation.z = 0.10

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_light_bulb()
    except rospy.ROSInterruptException:
        pass
