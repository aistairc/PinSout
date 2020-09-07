# import pcl
#
#
# original_data = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/3d-model.ply")
#
# print len(original_data.to_list())
#
# ran


import numpy as np
import pcl
import random
import os

def main():

    path = "/home/dprt/Documents/dprt/pointnet_data/2020_09_07/Untitled Folder/"

    # cloud = pcl.load(path+"3d-model_without_out.ply")
    cloud = pcl.load(path+"original_data.ply")

    # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/3d-model_wall_2.ply")

    cloud_size = cloud.size
    # print cloud_size
    # b = cloud.to_list()
    # a = cloud.extract([0,1,2])

    for i in range(0, 7):
        print cloud_size
        percentage = (i + 1) * 10
        max_point_count = cloud_size * percentage / 100
        sampling_count = 1000
        K = max_point_count / sampling_count

        # print cloud_size
        # sampling_count = 100
        # K = 1000
        #
        folder_name = str(sampling_count) + "_" + str(K)
        new_folder_path = path+folder_name
        try:
            os.mkdir(new_folder_path)
        except OSError:
            print ("Creation of the directory %s failed" % new_folder_path)
        else:
            print ("Successfully created the directory %s " % new_folder_path)

        # if cloud_size < sampling_count * K:
        #     check_value = str(int(cloud_size / sampling_count))
        #     print check_value
            # sampling_count = 10
            # K = 10

        kdtree = cloud.make_kdtree_flann()



        searchPoints = list()

        ran_num = random.randint(0, cloud_size)
        for i in range(sampling_count):
            # sampling_index = random.randrange(0, cloud_size)
            # searchPoints.append(sampling_index)
            while ran_num in searchPoints:
                ran_num = random.randint(0, cloud_size)
            searchPoints.append(ran_num)
        searchPoints.sort()
        # print searchPoints
        searchPoint = cloud.extract(searchPoints)


        [ind, sqdist] = kdtree.nearest_k_search_for_cloud(searchPoint, K)
        # if nearest_k_search_for_cloud

        result = list()
        for i in range(len(ind)):
            result.extend(ind[i])

        print len(result)

        in_cloud = cloud.extract(result, True)
        out_cloud = cloud.extract(result, False)

        pcl.save(in_cloud, new_folder_path+"/sampling_in_d.ply")
        pcl.save(out_cloud, new_folder_path+"/sampling_out_d.ply")
        # in_cloud._to_ply_file(new_folder_path+"/sampling_in_d.ply")
        # out_cloud._to_ply_file(new_folder_path+"/sampling_out_d.ply")

        #

        # radius = 0.05
        #
        # [ind, sqdist] = kdtree.radius_search_for_cloud(searchPoint, radius)
        # # if nearest_k_search_for_cloud
        # print ind
        # result = list()
        # for i in range(len(ind)):
        #     result.extend(ind[i])
        #
        # print len(result)
        #
        # in_cloud = cloud.extract(result, True)
        # out_cloud = cloud.extract(result, False)
        # print len(in_cloud), len(out_cloud)
        # in_cloud._to_ply_file("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/sampling_in_r.ply")
        # out_cloud._to_ply_file("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/sampling_out_r.ply")
        #
        # #

        # print('Neighbors within radius search at (' + str(searchPoint[0][0]) + ' ' + str(
        #     searchPoint[0][1]) + ' ' + str(searchPoint[0][2]) + ') with radius=' + str(radius))
        #
        #
        # [ind, sqdist] = kdtree.radius_search_for_cloud(searchPoint, radius)
        # for i in range(0, ind.size):
        #     print('(' + str(cloud[ind[0][i]][0]) + ' ' + str(cloud[ind[0][i]][1]) + ' ' + str(
        #         cloud[ind[0][i]][2]) + ' (squared distance: ' + str(sqdist[0][i]) + ')')


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()














# # import pcl
# #
# #
# # original_data = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/3d-model.ply")
# #
# # print len(original_data.to_list())
# #
# # ran
#
#
# import numpy as np
# import pcl
# import random
# import os
#
# def main():
#     path = "/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/"
#     # cloud = pcl.load(path+"3d-model_without_out.ply")
#     cloud = pcl.load(path+"test_pointcloud.ply")
#
#     # cloud = pcl.load("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/3d-model_wall_2.ply")
#
#     cloud_size = cloud.size
#     # print cloud_size
#     # b = cloud.to_list()
#     # a = cloud.extract([0,1,2])
#
#     for i in range(0, 5):
#         percentage = (i + 1) * 10
#         max_point_count = cloud_size * percentage / 100
#         sampling_count = 1000
#         K = max_point_count / sampling_count
#
#         # print cloud_size
#         # sampling_count = 100
#         # K = 1000
#         #
#         folder_name = str(sampling_count) + "_" + str(K)
#         new_folder_path = path+folder_name
#         try:
#             os.mkdir(new_folder_path)
#         except OSError:
#             print ("Creation of the directory %s failed" % new_folder_path)
#         else:
#             print ("Successfully created the directory %s " % new_folder_path)
#
#         # if cloud_size < sampling_count * K:
#         #     check_value = str(int(cloud_size / sampling_count))
#         #     print check_value
#             # sampling_count = 10
#             # K = 10
#
#         kdtree = cloud.make_kdtree_flann()
#
#
#
#         searchPoints = list()
#
#
#         for i in range(sampling_count):
#             sampling_index = random.randrange(0, cloud_size)
#             searchPoints.append(sampling_index)
#
#         searchPoint = cloud.extract(searchPoints)
#
#
#         [ind, sqdist] = kdtree.nearest_k_search_for_cloud(searchPoint, K)
#         # if nearest_k_search_for_cloud
#
#         result = list()
#         for i in range(len(ind)):
#             result.extend(ind[i])
#
#         print len(result)
#
#         in_cloud = cloud.extract(result, True)
#         out_cloud = cloud.extract(result, False)
#
#         in_cloud._to_ply_file(new_folder_path+"/sampling_in_d.ply")
#         out_cloud._to_ply_file(new_folder_path+"/sampling_out_d.ply")
#
#         #
#
#         # radius = 0.05
#         #
#         # [ind, sqdist] = kdtree.radius_search_for_cloud(searchPoint, radius)
#         # # if nearest_k_search_for_cloud
#         # print ind
#         # result = list()
#         # for i in range(len(ind)):
#         #     result.extend(ind[i])
#         #
#         # print len(result)
#         #
#         # in_cloud = cloud.extract(result, True)
#         # out_cloud = cloud.extract(result, False)
#         # print len(in_cloud), len(out_cloud)
#         # in_cloud._to_ply_file("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/sampling_in_r.ply")
#         # out_cloud._to_ply_file("/home/dprt/Documents/dprt/pointnet_data/3dModelPLY/test/sampling_out_r.ply")
#         #
#         # #
#
#         # print('Neighbors within radius search at (' + str(searchPoint[0][0]) + ' ' + str(
#         #     searchPoint[0][1]) + ' ' + str(searchPoint[0][2]) + ') with radius=' + str(radius))
#         #
#         #
#         # [ind, sqdist] = kdtree.radius_search_for_cloud(searchPoint, radius)
#         # for i in range(0, ind.size):
#         #     print('(' + str(cloud[ind[0][i]][0]) + ' ' + str(cloud[ind[0][i]][1]) + ' ' + str(
#         #         cloud[ind[0][i]][2]) + ' (squared distance: ' + str(sqdist[0][i]) + ')')
#
#
# if __name__ == "__main__":
#     # import cProfile
#     # cProfile.run('main()', sort='time')
#     main()