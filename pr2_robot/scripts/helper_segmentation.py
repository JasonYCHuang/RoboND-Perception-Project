# Import PCL module
import pcl
from sensor_stick.pcl_helper import *

save_file = False

# Voxel Grid filter.
def make_voxel(cloud, leaf_size = 0.005):
	vox = cloud.make_voxel_grid_filter()
	vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
	
	cloud_filtered = vox.filter()
	filename = 'voxel_downsampled.pcd'
	if save_file: pcl.save(cloud_filtered, filename)
	return cloud_filtered

# PassThrough filter
def make_pass_through(cloud, filter_axis = 'z', axis_min = 0.6, axis_max = 1.0):
	pass_through = cloud.make_passthrough_filter()
	pass_through.set_filter_field_name(filter_axis)
	pass_through.set_filter_limits(axis_min, axis_max)

	cloud_filtered = pass_through.filter()
	filename = 'pass_through_filtered.pcd'
	if save_file: pcl.save(cloud_filtered, filename)
	return cloud_filtered

# RANSAC plane segmentation
def make_ransac(cloud, max_dist = 0.01):
	seg = cloud.make_segmenter()

	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_model_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold(max_dist)

	inliers, coefficients = seg.segment()
	return inliers, coefficients

# Extract inliers
def extract_inliers(cloud, inliers, negative = False):
	extracted_inliers = cloud.extract(inliers, negative=negative)
	filename = 'extracted_inliers.pcd'
	if save_file: pcl.save(extracted_inliers, filename)
	return extracted_inliers

# Extract outliers
def std_filter_outlier(cloud):
	outlier_filter = cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(10)
	x = 0.01
	outlier_filter.set_std_dev_mul_thresh(x)
	cloud_filtered = outlier_filter.filter()
	filename = 'filter_outlier.pcd'
	if save_file: pcl.save(cloud_filtered, filename)
	return cloud_filtered

def execute_segmentation(pcl_msg):
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    cloud_vx = make_voxel(cloud)

    # TODO: PassThrough Filter
    cloud_vx_psz = make_pass_through(cloud_vx, 'z', 0.6, 1.1)
    cloud_vx_pszy = make_pass_through(cloud_vx_psz, 'y', -0.5, 0.5)

    # TODO: Statistical Outlier Filtering
    cloud_vx_pszy_std = std_filter_outlier(cloud_vx_pszy)

    # TODO: RANSAC Plane Segmentation
    inliers, coefficients = make_ransac(cloud_vx_pszy_std)

    # TODO: Extract inliers and outliers
    cloud_table = extract_inliers(cloud_vx_pszy_std, inliers, False)
    cloud_objects = extract_inliers(cloud_vx_pszy_std, inliers, True)
    return cloud_table, cloud_objects
