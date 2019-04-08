#!/usr/bin/env python
import rospy

import numpy as numpy
import numpy.linalg as la
import tf
import tf2_ros
import geometry_msgs.msg

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)

import tf_conversions

def publish_transforms():

    # World frame to tag1 frame transform

    # tag_tf = geometry_msgs.msg.TransformStamped()
    # tag_tf.header.stamp = rospy.Time.now()
    # tag_tf.header.frame_id = "world"
    # tag_tf.child_frame_id = "tag1"
    # tag_tf.transform.translation.x = 1.0
    # tag_tf.transform.translation.y = 0.0
    # tag_tf.transform.translation.z = 0.0
    # q_tag = tf.transformations.quaternion_from_euler(0, 0, 0)
    # tag_tf.transform.rotation.x = q_tag[0]
    # tag_tf.transform.rotation.y = q_tag[1]
    # tag_tf.transform.rotation.z = q_tag[2]
    # tag_tf.transform.rotation.w = q_tag[3]
    # br.sendTransform(tag_tf)

    # Camera frame to tag frame transform

    tag_tf = geometry_msgs.msg.TransformStamped()
    tag_tf.header.stamp = rospy.Time.now()
    tag_tf.header.frame_id = "camera"
    tag_tf.child_frame_id = "tag"
    tag_tf.transform.translation.x = 1.0
    tag_tf.transform.translation.y = 0.0
    tag_tf.transform.translation.z = 0.0
    q_tag = tf.transformations.quaternion_from_euler(0, 0, 0)
    tag_tf.transform.rotation.x = q_tag[0]
    tag_tf.transform.rotation.y = q_tag[1]
    tag_tf.transform.rotation.z = q_tag[2]
    tag_tf.transform.rotation.w = q_tag[3]
    br.sendTransform(tag_tf)

def tag_pose_callback(pose_array):

    # World frame to tag frame(s)

    # Camera frame to tag frame(s)
    if (len(pose_array.detections)==0):
        return

    for i in range(len(pose_array.detections)):
        pose = pose_array.detections[i].pose.pose
        tag_tf = geometry_msgs.msg.TransformStamped()
        tag_tf.header.stamp = rospy.Time.now()
        tag_tf.header.frame_id = "camera"
        tag_tf.child_frame_id = "tag" + str(i)
        tag_tf.transform.translation.x = pose.position.x
        tag_tf.transform.translation.y = pose.position.y
        tag_tf.transform.translation.z = pose.position.z
        tag_tf.transform.rotation.x = pose.orientation.x
        tag_tf.transform.rotation.y = pose.orientation.y
        tag_tf.transform.rotation.z = pose.orientation.z
        tag_tf.transform.rotation.w = pose.orientation.w
        br.sendTransform(tag_tf)

if __name__ == '__main__':
    rospy.init_node('cam2tag_broadcaster')

    sub = rospy.Subscriber("/camera/tag_detections",AprilTagDetectionArray,tag_pose_callback)
    br = tf2_ros.TransformBroadcaster()

    rospy.spin()

    # rospy.sleep(0.5)

    # while not rospy.is_shutdown():
    #     publish_transforms()
    #     rospy.sleep(0.05)
