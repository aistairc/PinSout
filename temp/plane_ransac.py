import pcl
import pcl.pcl_visualization
import math
import random


def do_passthrough_filter( point_cloud, name_axis='z'):
    pass_filter = point_cloud.make_passthrough_filter()
    pass_filter.set_filter_field_name(name_axis)

    return pass_filter.filter()


def do_ransac_plane_segmentation(point_cloud, ksearch):

    segmenter = point_cloud.make_segmenter_normals(ksearch=ksearch)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)

    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(1000)
    segmenter.set_distance_threshold(0.1)

    inlier_indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(inlier_indices, negative=False)

    outliers = point_cloud.extract(inlier_indices, negative=True)

    return inliers, outliers, coefficients


def check_distance_plane(point_cloud, coeff):

    a = coeff[0]
    b = coeff[1]
    c = coeff[2]
    d = coeff[3]
    t = point_cloud.to_list()
    count = 0
    for index in t:
        x = index[0]
        y = index[1]
        z = index[2]

        point_distance = float(
            math.fabs(a * x + b * y + c * z + d) / math.sqrt(math.pow(a, 2) + math.pow(b, 2) + math.pow(c, 2)))

        if point_distance <= 0.1:
            count += 1

    if count == 0:
        distance_rate = 0.0
    else:
        distance_rate = round(float(count) / float(len(t)), 3)
    return distance_rate

def get_normal_vector(cloud, ksearch=50):
    print "hi"
    min_percentage = 10
    plane_list = list()

    original_size = cloud.size

    while (cloud.height * cloud.width > original_size * min_percentage / 100):

        filtered_cloud = do_passthrough_filter(point_cloud=cloud, name_axis='z')
        inliers_p, outliers_p, coeff_p = do_ransac_plane_segmentation(filtered_cloud, ksearch)

        plane_list.append(inliers_p)
        cloud = outliers_p

        print len(plane_list)
        if cloud.height * cloud.width < original_size * min_percentage / 100:
            break


    return plane_list

def visual_viewer(cloud_list):

    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

    viewer.SetBackgroundColor(0, 0, 0)

    viewer.AddPointCloud(cloud, b'sample cloud')
    viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'sample cloud')


    for index in range(len(cloud_list)):
        r = random.randrange(0, 256)
        b = random.randrange(0, 256)
        g = random.randrange(0, 256)

        point_size = 10
        color_handle = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_list[index], r, b, g)
        id = b'inliers_cloud_' + str(index)
        viewer.AddPointCloud_ColorHandler(cloud_list[index], color_handle, id)
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, point_size, id)

    viewer.InitCameraParameters()
    while viewer.WasStopped() != True:
        viewer.SpinOnce(100)


if __name__ == "__main__":
    cloud = pcl.load("/home/dprt/Desktop/SC_DEMO/New Folder/test/conferenceRoom_1_wall.ply")
    print cloud
    result = get_normal_vector(cloud)
    #
    visual_viewer(result)

