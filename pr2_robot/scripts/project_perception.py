#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from pr2_robot.srv import *
import yaml

from helper_segmentation import *
from helper_clustering import *
from helper_recognition import *
from helper_pp_service_content import *

TEST_SCENE_NUM = 2
MOVE_ROBOT = False

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
# Exercise-2 TODOs:
    cloud_table, cloud_objects = execute_segmentation(pcl_msg)
    white_cloud, cluster_cloud, cluster_indices = execute_clustering(cloud_objects)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud_objects = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud_objects)

# Exercise-3 TODOs:
    detected_objects = execute_recognition(
        clf, 
        scaler, 
        encoder,
        object_markers_pub,
        cloud_objects, 
        cluster_indices,
        white_cloud
    )

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass


# function to load parameters and request PickPlace service
def pr2_mover(object_list):
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    yaml_dict_list = []
    for i in range(0, len(object_list_param)):
        test_scene_num, arm_name, object_name, pick_pose, place_pose = gen_pp_service_content(
            i, 
            object_list_param, 
            dropbox_param, 
            object_list,
            TEST_SCENE_NUM
        )

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        yaml_dict_list.append(yaml_dict)

        if MOVE_ROBOT:
            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')
            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # TODO: Insert your message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    filename = './result/output_%s.yaml' % TEST_SCENE_NUM
    send_to_yaml(filename, yaml_dict_list)
    print 'Save %s successfully!' % filename


if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber(
        '/pr2/world/points', 
        PointCloud2, 
        pcl_callback, 
        queue_size=1
    )

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher(
        '/pcl_objects', 
        PointCloud2, 
        queue_size=1
    )

    pcl_table_pub = rospy.Publisher(
        '/pcl_table', 
        PointCloud2, 
        queue_size=1
    )

    pcl_cluster_pub = rospy.Publisher(
        '/pcl_cluster', 
        PointCloud2, 
        queue_size=1
    )

    object_markers_pub = rospy.Publisher(
        '/object_markers', 
        Marker, 
        queue_size=1
    )

    detected_objects_pub = rospy.Publisher(
        '/detected_objects', 
        DetectedObjectsArray, 
        queue_size=1
    )

    # TODO: Load Model From disk
    clf, encoder, scaler = load_prediction_model(TEST_SCENE_NUM)

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
