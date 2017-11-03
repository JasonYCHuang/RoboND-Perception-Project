import pickle

from sklearn.preprocessing import LabelEncoder
from sensor_stick.pcl_helper import *
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.msg import DetectedObject
from sensor_stick.srv import GetNormals
from sensor_stick.marker_tools import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def load_prediction_model(scene_num):
    filename = './train_models/model_%s.sav' % scene_num
    model = pickle.load(open(filename, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    return clf, encoder, scaler

def create_feature(pcl_cluster_cloud):
    chists = compute_color_histograms(pcl_cluster_cloud, using_hsv=True)
    normals = get_normals(pcl_cluster_cloud)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists, nhists))
    return feature

def make_prediction(clf, scaler, encoder, feature):
    prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
    label = encoder.inverse_transform(prediction)[0]
    return label

def detect_object(label, pcl_cluster_cloud):
    do = DetectedObject()
    do.label = label
    do.cloud = pcl_cluster_cloud
    return do

def execute_recognition(clf, scaler, encoder, object_markers_pub, cloud_objects, cluster_indices, white_cloud):
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        pcl_cluster_cloud = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        feature = create_feature(pcl_cluster_cloud)

        # Make the prediction
        label = make_prediction(clf, scaler, encoder, feature)
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = detect_object(label, pcl_cluster_cloud)
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    return detected_objects
