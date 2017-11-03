## Project: Perception Pick & Place

---

[//]: # (Image References)

[vox]: ./supporting_info/vox.png
[ps]: ./supporting_info/ps.png
[std_compare]: ./supporting_info/std_compare.png
[ransac]: ./supporting_info/ransac.png
[clustering]: ./supporting_info/clustering.png
[scene_1_model_1]: ./pr2_robot/scripts/model/pick_list_1/figure_1.png
[scene_1_model_2]: ./pr2_robot/scripts/model/pick_list_1/figure_2.png
[scene_2_model_1]: ./pr2_robot/scripts/model/pick_list_2/figure_1.png
[scene_2_model_2]: ./pr2_robot/scripts/model/pick_list_2/figure_2.png
[scene_3_model_1]: ./pr2_robot/scripts/model/pick_list_3/figure_1.png
[scene_3_model_2]: ./pr2_robot/scripts/model/pick_list_3/figure_2.png
[result_1]: ./supporting_info/result_1.png
[result_2]: ./supporting_info/result_2.png
[result_3]: ./supporting_info/result_3.png
[prediction]: ./supporting_info/prediction.png

This project can be divided into 4 parts:

1. Use filters and RANSAC plane fitting to get table and objects from point cloud data.
2. Use Euclidean clustering to cluster objects.
3. Use color and normal features with SVM to predicts which clustered cloud data belongs to what kind of objects.
4. Output results to YAML files, and this information will be the response of the `pick_place_routine` service, and provide objects position & destinations for motion planning.

---

### 0. Point Cloud data

A traditional camera is a device to record visible light projection to a 2D plane. However, our world is a 3 dimention space, and the projection from 3D to 2D will lead to information loss. Here, I use a RGB-D camera to get visible light and keep 3D space mapping. Therefore, __OPENCV__ library is not the best candidate for 3D vision tool, and I will implement __Point Cloud Library__ with handy tools to deal with stereoscopic problems.

To get data from the RGB-D camera, I need to [subscribe](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L131) `/pr2/world/points`. Once I get data, it will invoke `pcl_callback` to process point cloud data.

```
pcl_sub = rospy.Subscriber(
    '/pr2/world/points', 
    PointCloud2, 
    pcl_callback, 
    queue_size=1
)
```

The RGB-D camera output format is `PointXYZRGB`, and I need to use `PointCloud2` format in ROS. The [conversion](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/pcl_helper.py#L59) is already implemented in `pcl_helper.py`.

### 1. [Segmentation](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L44)

I implement this part in function [`execute_segmentation`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_segmentation.py#L57). After converted point cloud data to `PointCloud2`, I use Voxel, PassThrough, Statistical Outlier filters. 

The [__Voxel filter__](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_segmentation.py#L8) is down-sampling, and the purpose is to reduce computation cost. For example, assuming I sample 10 points on 1cm, and total sampling points in 1 cm<sup>3</sup> is 1000. If I sample 2 points on 1cm, and total sampling points will be 8 points. Although this reduce computation cost, it also reduces information for later stages. I need to find a balance between down-sampling and maintaining recognition abilities. 

Here is the result of Voxel filter:

![alt text][vox]

[__PassThrough filters__](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_segmentation.py#L18) can cut-off data out of ROI(region of interest). In the exercise, I use only a Z-axis PassThrough filter, to remove data under the table. However, I need addition a Y-axis PassThrough filter to remove data on the side. Otherwise, things not in ROI will also be force recognized into our models, and result in the wrong recognition.

```
cloud_vx_psz = make_pass_through(cloud_vx, 'z', 0.6, 1.1)
cloud_vx_pszy = make_pass_through(cloud_vx_psz, 'y', -0.5, 0.5)
```

The left image is only a z-axis PassThrough filter, but objects on sides can not removed; the right one is a z-and-y PassThrough filter which removes objects on sides.

![alt text][ps]

[__Statistical Outlier Filters__](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_segmentation.py#L47) remove noisy data by computing mean distance from each points to all its neighbors. When this mean distance is larger than a threshold, the point is consider an isolated outlier. Therefore, in the code, I need to set threshold and a number of neighbors.

```
def std_filter_outlier(cloud):
	outlier_filter = cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(10)
	x = 0.01
	outlier_filter.set_std_dev_mul_thresh(x)
	cloud_filtered = outlier_filter.filter()
	...
```

Here are compared images. (Left: noisy; Right: remove noise with Statistical Outlier Filter)

![alt text][std_compare]

__RANSAC__ requires the proior knowledge for the target, and iterate through whole datasets verify and distinguis inliers and outliers. The process is as follows:

1. Pick `N` randomly selected points from dataset to create a model.
2. `n` Points in `N` fit the model within an error threshold, they are considered as inliers.
3. Take the rest points from datasets. If points also fit the model within an error threshold, they are also added into inliers.
4. Calculate error of fitting inliers into the model
5. Save this model and inliers when inliers is larger than a good threshold.
6. Repeat 1-5, and get the best model for the lowest error.

I don't have to implement the detail, and just use the [library](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_segmentation.py#L29):

```
def make_ransac(cloud, max_dist = 0.01):
	seg = cloud.make_segmenter()

	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_model_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold(max_dist)

	inliers, coefficients = seg.segment()
	...
```

Here are segmentation results.

![alt text][ransac]

### 2. [Clustering](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L45)

Point cloud data is now filtered to remove noise, but points belong to which objects are not known.

Clustering is a task to assgin points in the space to groups, and we expect groups corresponding to our objects. We can apply Euclidean clustering. If distance of a point to its neighbors is smalller than a threshold, they are belong to the same group. Here, since we only consider distance, the spatial information of points are the data for Clustering. Color information is of no usage, because an object usually contains several colors. I retrieve only spatial information with [`XYZRGB_to_XYZ`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_clustering.py#L33). 

To use Euclidean clustering, we need to compute distance to neightbors. This means for a point, I need to search through all datasets, find neighbors and calculate mean distance. This requires heavy computing. Therefore, [k-d tree](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_clustering.py#L5) is a binary tree splitting method, and it is adopted to decrease computation cost.

Here is a picture of clustering.

![alt text][clustering]

### 3. [Object recognition](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L58)

Here, SVM(support vector machine) is adopted to classify groups base on color and surface normal histogram features. SVM divides data space base on their classes, and finds boundaries to separate different classes. Data points close to boundaries are called support vectors, and SVM finds hyperplanes which separate diffent classes with largest margins among support vectors. To let SVM to learn, we need to provide training sets, and I use `create_feature.py` in ecercise-3 to record histogram features for different orientations. By running `train_svm.py` in exercise-3, I can get trained models.

During training, two factors can boost accuracy: [1]prefer HSV over RGB; [2]more examples in training sets. HSV decouples color(hue) and intensity, and we still can know color when low light intensity. When creating features, if I use more examples per objects, I will get higher accuracy in the training. In the end, I use 20 exmples for each objects. The drawback is time consuming, but, luckily, we only do this once during training, and the [predictions](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_recognition.py#L42) speed is not compromised.

Scene 1 model accuracy: 96.7%

![alt text][result_1]

Scene 2 model accuracy: 97.0%

![alt text][result_2]

Scene 3 model accuracy: 94.3%

![alt text][result_3]

In order to make predictions, I need to call function [`load_prediction_model(TEST_SCENE_NUM)`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L170) to load model parameters, and function [`make_prediction`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_recognition.py#L31) uses them to predict a label. `detected_objects` with labels will be [published](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L69), and I can see labels in the Rviz.

![alt text][prediction]

### 4. Output results

At the final stage, I conclude the result, and svae it to YAML files. This information will be the response of the [`pick_place_routine`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L110) service. There are default parameters in ROS parameter server, and I can [read them](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L84) with:

```
object_list_param = rospy.get_param('/object_list')
dropbox_param = rospy.get_param('/dropbox')
```

Looping through `object_list_param`, function [`gen_pp_service_content`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_pp_service_content.py#L46) composes YAML content and save to files. By [comparing name](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_pp_service_content.py#L17) from `object_list_param` and recognized objects, we can get specific poin cloud groups. [Centroids](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_pp_service_content.py#L23) are positions of these points which are averaged and are converted to native python type `float` using [`np.asscalar()`](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_pp_service_content.py#L26). With `dropbox_param`, we can know the placement positions by [comparing object groups](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/helper_pp_service_content.py#L11) to `object_param` groups. The data is converted to proper format: `Int32`, `String`, `Float64` and `Pose`. Finally, they are [saved to YAML files](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/project_perception.py#L122), and the result is here [YAML1](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/result/output_1.yaml), [YAML2](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/result/output_2.yaml) and [YAML3](https://github.com/JasonYCHuang/RoboND-Perception-Project/blob/master/pr2_robot/scripts/result/output_3.yaml).




