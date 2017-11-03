import pcl
import numpy as np

from sensor_stick.pcl_helper import *
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String

def matched_arm(target_group, default_arms):
    for arm in default_arms:
        if arm['group'] == target_group:
            return arm
    return None

def matched_cloud_object(target_label, cloud_objects):
    for obj in cloud_objects:
        if obj.label == target_label:
            return obj
    return None

def cloud_to_centroid(cloud):
    points_arr = ros_to_pcl(cloud).to_array()
    centroid = np.mean(points_arr, axis=0)[:3]
    centroid = [np.asscalar(c) for c in centroid]
    return centroid

def set_position(pos):
    target_pose = Pose()
    target_pose.position.x = pos[0]
    target_pose.position.y = pos[1]
    target_pose.position.z = pos[2]
    return target_pose

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

def gen_pp_service_content(idx, object_list_param, dropbox_param, object_list, scene_num):
    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # TODO: Get scene num
    test_scene_num.data = scene_num

    # TODO: Get object name
    target_name = object_list_param[idx]['name']
    object_name.data = target_name

    # TODO: Get the PointCloud for a given object and obtain it's centroid
    target_object = matched_cloud_object(target_name, object_list)
    target_centroid = cloud_to_centroid(target_object.cloud)

    # TODO: Create 'pick_pose' for the object
    pick_pose = set_position(target_centroid)

    # TODO: Assign the arm to be used for pick_place
    target_group = object_list_param[idx]['group']
    target_arm = matched_arm(target_group, dropbox_param)
    arm_name.data = target_arm['name']
    place_pose = set_position(target_arm['position'])

    return test_scene_num, arm_name, object_name, pick_pose, place_pose