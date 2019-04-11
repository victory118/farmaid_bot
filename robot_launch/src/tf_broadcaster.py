#!/usr/bin/env python
import rospy

import sys

import yaml
import numpy as np
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

class TfBroadcaster:
    def __init__(self, tag_info, pose_robot2camera):
        # Initialize broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Initialize subscriber
        self.sub = rospy.Subscriber("/camera/tag_detections", AprilTagDetectionArray, self.callback)

        # AprilTag info: world frame to tag frame transform
        self.tag_info = tag_info # dictionary with tag_id: [x, y, z, qx, qy, qz, qw]

        # Pose of camera in the robot frame
        self.pose_robot2camera = pose_robot2camera # list with pose info: [x, y, z, qx, qy, qz, qw]

    def broadcast(self):
        """
        Broadcast world frame to tag frame transforms
        """

        for tag_id, tf_data in self.tag_info.iteritems():
            tf_w2t = geometry_msgs.msg.TransformStamped()
            tf_w2t.header.stamp = rospy.Time.now()
            tf_w2t.header.frame_id = "world"
            tf_w2t.child_frame_id = "tag" + str(tag_id)
            tf_w2t.transform.translation.x = tf_data[0]
            tf_w2t.transform.translation.y = tf_data[1]
            tf_w2t.transform.translation.z = tf_data[2]
            tf_w2t.transform.rotation.x = tf_data[3]
            tf_w2t.transform.rotation.y = tf_data[4]
            tf_w2t.transform.rotation.z = tf_data[5]
            tf_w2t.transform.rotation.w = tf_data[6]
            self.br.sendTransform(tf_w2t)

    def callback(self, pose_array):
        """
        Broadcast camera frame to tag frame transforms and
        calculate world frame to camera frame
        """
        # Camera frame to tag frame(s)
        if (len(pose_array.detections)==0):
            return

        for i in range(len(pose_array.detections)):
            pose = pose_array.detections[i].pose.pose
            tag_id = pose_array.detections[i].id
            tf_c2t = geometry_msgs.msg.TransformStamped()
            tf_c2t.header.stamp = rospy.Time.now()
            tf_c2t.header.frame_id = "camera"
            tf_c2t.child_frame_id = "tag" + str(tag_id)
            tf_c2t.transform.translation.x = pose.position.x
            tf_c2t.transform.translation.y = pose.position.y
            tf_c2t.transform.translation.z = pose.position.z
            tf_c2t.transform.rotation.x = pose.orientation.x
            tf_c2t.transform.rotation.y = pose.orientation.y
            tf_c2t.transform.rotation.z = pose.orientation.z
            tf_c2t.transform.rotation.w = pose.orientation.w
            self.br.sendTransform(tf_c2t)

            # Calculate world to camera transform
            translation_c2t = [pose.position.x, pose.position.y, pose.position.z]
            quat_c2t = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            T_c2t = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(translation_c2t),
                    tf.transformations.quaternion_matrix(quat_c2t))
            
            translation_w2t = self.tag_info[tag_id][:3]
            quat_w2t = self.tag_info[tag_id][3:]
            T_w2t = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(translation_w2t),
                    tf.transformations.quaternion_matrix(quat_w2t))

            T_w2c = np.dot(T_w2t, tf.transformations.inverse_matrix(T_c2t))

            tf_w2c = geometry_msgs.msg.TransformStamped()
            tf_w2c.header.stamp = rospy.Time.now()
            tf_w2c.header.frame_id = "world"
            tf_w2c.child_frame_id = "camera"
            tf_w2c.transform = message_from_transform(T_w2c)
            self.br.sendTransform(tf_w2c)

            # Calculate world to robot transform
            translation_r2c = self.pose_robot2camera[:3]
            quat_r2c = self.pose_robot2camera[3:]

            T_r2c = tf.transformations.concatenate_matrices(
                    tf.transformations.translation_matrix(translation_r2c),
                    tf.transformations.quaternion_matrix(quat_r2c))

            T_w2r = np.dot(T_w2c, tf.transformations.inverse_matrix(T_r2c))

            tf_w2r = geometry_msgs.msg.TransformStamped()
            tf_w2r.header.stamp = rospy.Time.now()
            tf_w2r.header.frame_id = "world"
            tf_w2r.child_frame_id = "robot"
            tf_w2r.transform = message_from_transform(T_w2r)
            self.br.sendTransform(tf_w2r)   

def message_from_transform(T):
	'''
	Input: 4x4 Homogeneous transform matrix
    Output: Transform message
	'''
	msg = geometry_msgs.msg.Transform()
	q = tf.transformations.quaternion_from_matrix(T)
	translation = tf.transformations.translation_from_matrix(T)
	msg.translation.x = translation[0]
	msg.translation.y = translation[1]
	msg.translation.z = translation[2]
	msg.rotation.x = q[0]
	msg.rotation.y = q[1]
	msg.rotation.z = q[2]
	msg.rotation.w = q[3]
	return msg

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    # Load tag_info parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()

    params = yaml.load(params_raw)
    tag_info = params['tag_info']
    print "tag_info = ", tag_info

    pose_robot2camera = params['pose_robot2camera']
    print "pose_robot2camera = ", pose_robot2camera

    # sub = rospy.Subscriber("/camera/tag_detections",AprilTagDetectionArray,tag_pose_callback)
    # br = tf2_ros.TransformBroadcaster()
    
    # rospy.spin()

    tf_br = TfBroadcaster(tag_info, pose_robot2camera)
    rospy.sleep(0.5)

    rate = rospy.Rate(20) # 20 Hz

    while not rospy.is_shutdown():
        # publish_transforms()
        tf_br.broadcast()
        rate.sleep()
        # rospy.sleep(0.05)
