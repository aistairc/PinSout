import argparse
import os
import sys
import time
import pcl
from src.SC_demo.plane_ransac_test import  *
import src.SC_demo.ply2obj as po
# import plane_ransac_test as pr

# import csv
# import json
# import numpy as np
# import tensorflow as tf
# import pcl as pc
# from collections import OrderedDict
# import datetime

from model import *
import indoor3d_util2
''' New version '''
BATCH_SIZE = 1
NUM_POINT = 4096
GPU_INDEX = 0
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
NUM_CLASSES = 6

def evaluate(model_path, out_filename, booth_data, min_list):
    '''

    :param model_path: trained model path
    :param filepath: file stored dir
    :param booth_data: point cloud data type : Array / value [x, y, z, r, g, b, label(5)]
    :param xyz_min: min xyz of ply data(all)
    :param min_list: min xyz of each npy data
    :return: 3D model path, bounding box area of chair data [max point(x, y, z), min point(x, y, z]
    '''
    is_training = False

    # BATCH_SIZE = 1
    # NUM_POINT = 4096
    # GPU_INDEX = 0
    MODEL_PATH = model_path

    DUMP_DIR = os.path.join(os.path.dirname(out_filename), 'dump')
    if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
    # ROOM_PATH_LIST = [os.path.join(ROOT_DIR, line.rstrip()) for line in open(room_data_filelist)]

    with tf.device('/gpu:' + str(GPU_INDEX)):
        pointclouds_pl, labels_pl = placeholder_inputs(BATCH_SIZE, NUM_POINT)
        is_training_pl = tf.placeholder(tf.bool, shape=())

        # simple model
        pred = get_model(pointclouds_pl, is_training_pl)
        loss = get_loss(pred, labels_pl)
        pred_softmax = tf.nn.softmax(pred)

        # Add ops to save and restore all the variables.
        saver = tf.train.Saver()

    # Create a session
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    config.allow_soft_placement = True
    config.log_device_placement = True
    sess = tf.Session(config=config)

    # Restore variables from disk.
    saver.restore(sess, MODEL_PATH)


    ops = {'pointclouds_pl': pointclouds_pl,
           'labels_pl': labels_pl,
           'is_training_pl': is_training_pl,
           'pred': pred,
           'pred_softmax': pred_softmax,
           'loss': loss}
    all_ceiling = list()
    all_floor = list()
    all_wall = list()
    all_chair = list()
    all_table = list()
    all_clutter = list()
    # scaned_data = np.load(ROOM_PATH_LIST[0])



    for each_i in range(len(booth_data)):


        ceiling_list, floor_list, wall_list, chair_list, table_list, clutter_list = eval_one_epoch(sess, ops, booth_data[each_i])
        if len(ceiling_list) != 0:
            temp_value_1 = np.asarray(ceiling_list)
            temp_value_1 += min_list[each_i]
            ceiling_cloud = pcl.PointCloud()
            ceiling_cloud.from_list(temp_value_1.tolist())
            pcl.save(ceiling_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_ceiling' + "_" + str(each_i)+".pcd")
        if len(floor_list) != 0:
            temp_value_2 = np.asarray(floor_list)
            temp_value_2 += min_list[each_i]
            floor_cloud = pcl.PointCloud()
            floor_cloud.from_list(temp_value_2.tolist())
            pcl.save(floor_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_floor' + "_" + str(each_i)+".pcd")
        if len(wall_list) != 0:
            temp_value_3 = np.asarray(wall_list)
            temp_value_3 += min_list[each_i]
            wall_cloud = pcl.PointCloud()
            wall_cloud.from_list(temp_value_3.tolist())
            pcl.save(wall_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_wall' + "_" + str(each_i)+".pcd")
            all_wall += temp_value_3.tolist()
        if len(chair_list) != 0:
            temp_value_4 = np.asarray(chair_list)
            temp_value_4 += min_list[each_i]
            chair_cloud = pcl.PointCloud()
            chair_cloud.from_list(temp_value_4.tolist())
            pcl.save(chair_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) +'_chair' + "_" + str(each_i)+".pcd")
        if len(table_list) != 0:
            temp_value_5 = np.asarray(table_list)
            temp_value_5 += min_list[each_i]
            table_cloud = pcl.PointCloud()
            table_cloud.from_list(temp_value_5.tolist())
            pcl.save(table_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_table' + "_" + str(each_i)+".pcd")
        if len(clutter_list) != 0:
            temp_value_6 = np.asarray(wall_list)
            temp_value_6 += min_list[each_i]
            clutter_cloud = pcl.PointCloud()
            clutter_cloud.from_list(temp_value_6.tolist())
            all_wall += temp_value_6.tolist()
            pcl.save(clutter_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + "_clutter" + "_" + str(each_i)+".pcd")
    print len(all_wall)
    ''' Third Process '''



    wall_cloud = pcl.PointCloud()
    wall_cloud.from_list(all_wall)
    wall_surface_list = make_wall_info(wall_cloud)
    #
    # dae_filename = os.path.join(DUMP_DIR, os.path.basename(out_filename))
    # dae_filename = dae_filename + '.dae'
    #
    # new_wall_data = np.asarray(all_wall, dtype=np.float32)
    # new_wall_data[:, 0:3] += xyz_min
    # wall_cloud = pcl.PointCloud()
    # wall_cloud.from_array(new_wall_data)
    # print DUMP_DIR,out_filename
    pcl.save(wall_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename))+'_wall.pcd')

    # chair_bbox_list = list()
    # print len(all_chair)
    # if len(all_chair) != 0:
    #
    #
    #     chair_cloud = pcl.PointCloud()
    #     chair_cloud.from_list(all_chair)
    #     save_path = os.path.join(DUMP_DIR, os.path.basename(out_filename))
    #     chair_bbox_list = make_chair_info(chair_cloud, xyz_min, save_path)
    #
    #     new_chair_data = np.asarray(all_chair, dtype=np.float32)
    #     new_chair_data[:, 0:3] += xyz_min
    #     chair_cloud = pcl.PointCloud()
    #     chair_cloud.from_list(new_chair_data)
    #     pcl.save(chair_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename))+'_chair.pcd')
    # else:
    #     print "There is no chairs in here!!"
    # #
    # table_bbox_list = list()
    # if len(all_table) != 0:
    #     table_cloud = pcl.PointCloud()
    #     table_cloud.from_list(all_table)
    #     save_path = os.path.join(DUMP_DIR, os.path.basename(out_filename))
    #     table_bbox_list = make_table_info(table_cloud, xyz_min, save_path)
    #
    #     temp_array = np.asarray(all_table, dtype=np.float32)
    #     temp_array[:, 0:3] += xyz_min
    #     table_cloud = pcl.PointCloud()
    #     table_cloud.from_list(temp_array)
    #     pcl.save(table_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_table.pcd')
    # else:
    #     print "There is no tables in here!!"
    #     table_bbox_list.append([])

    # ''' Fourth Process '''
    # poly2obj = po.Ply2Obj(wall_surface_list)
    # poly2obj.poly_2_obj('All')
    # poly2obj.output_dae(dae_filename)
    #
    # print dae_filename
    # return [dae_filename, chair_bbox_list]



def eval_one_epoch(sess, ops, each_data):

    is_training = False
    ceiling_list = list()
    floor_list = list()
    wall_list = list()
    chair_list = list()
    table_list = list()
    clutter_list = list()

    current_data, current_label = indoor3d_util2.room2blocks_wrapper_normalized(each_data, NUM_POINT)
    current_data = current_data[:, 0:NUM_POINT, :]
    print "-----------------------------------------------------------------"
    print current_data.shape, current_data.shape[0], current_data.shape[1]
    print current_label.shape, current_label.shape[0], current_label.shape[1]
    # if current_label.shape[0] != current_data.shape[0]:
    #     current_label = np.squeeze(current_label)
    # current_label = np.squeeze(current_label)

    # Get room dimension..
    data_label = each_data
    data = data_label[:, 0:6]
    max_room_x = max(data[:, 0])
    max_room_y = max(data[:, 1])
    max_room_z = max(data[:, 2])
    file_size = current_data.shape[0]
    num_batches = file_size // BATCH_SIZE
    print file_size
    if file_size >= 1:
        print "hi"
        for batch_idx in range(num_batches):
            start_idx = batch_idx * BATCH_SIZE
            end_idx = (batch_idx + 1) * BATCH_SIZE
            feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                         ops['labels_pl']: current_label[start_idx:end_idx],
                         ops['is_training_pl']: is_training}

            pred_val = sess.run(ops['pred_softmax'], feed_dict=feed_dict)

            # pred_label = np.argmax(pred_val[:, :, 0:5], 2)  # BxN
            pred_label = np.argmax(pred_val, 2)  # BxN
            for b in range(BATCH_SIZE):
                pts = current_data[start_idx + b, :, :]
                pts[:, 6] *= max_room_x
                pts[:, 7] *= max_room_y
                pts[:, 8] *= max_room_z
                pred = pred_label[b, :]
                for i in range(NUM_POINT):
                    if pred[i] == 0:
                        ceiling_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                    if pred[i] == 1:
                        floor_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                    if pred[i] == 2:
                        wall_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                    if pred[i] == 3:
                        table_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                    if pred[i] == 4:
                        chair_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                    if pred[i] == 5:
                        clutter_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
        print len(ceiling_list), len(floor_list), len(wall_list), len(chair_list), len(table_list), len(clutter_list)
        return ceiling_list, floor_list, wall_list, chair_list, table_list, clutter_list
    else:
        return [], [], [], [], [], []
if __name__ == '__main__':
    with tf.Graph().as_default():
        evaluate(model_path='/root/pointnet/sem_seg/log_6cls_test16/model.ckpt', out_filename = '/home/dprt/Desktop/SC_DEMO/New Folder/npy_data2/dump',
    booth_data= '/home/dprt/Desktop/SC_DEMO/room2.npy', xyz_min=np.asarray([0.0, 0.0, 0.0]))
# ''' Old version '''
# parser = argparse.ArgumentParser()
# parser.add_argument('--gpu', type=int, default=0, help='GPU to use [default: GPU 0]')
# parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during training [default: 1]')
# parser.add_argument('--num_point', type=int, default=8192, help='Point number [default: 4096]')
# # parser.add_argument('--model_path', required=True, help='model checkpoint file path')
# # parser.add_argument('--dump_dir', required=True, help='dump folder path')
# # parser.add_argument('--output_filelist', required=True,
# #                     help='TXT filename, filelist, each line is an output for a room')
# # parser.add_argument('--room_data_filelist', required=True,
# #                     help='TXT filename, filelist, each line is a test room data label file.')
# parser.add_argument('--no_clutter', action='store_true', help='If true, donot count the clutter class')
# # parser.add_argument('--visu', action='store_true', help='Whether to output OBJ file for prediction visualization.')
# FLAGS = parser.parse_args()
#

#
# def evaluate(model_path, dump_dir, room_data_filelist, xyz_min):
#     ''' '''
#     BASE_DIR = os.path.dirname(os.path.abspath(__file__))
#     ROOT_DIR = os.path.dirname(BASE_DIR)
#     sys.path.append(BASE_DIR)
#     # BATCH_SIZE = 1
#     # NUM_POINT = 4096
#     # GPU_INDEX = 0
#     MODEL_PATH = model_path
#     DUMP_DIR = dump_dir
#     NUM_CLASSES = 6
#     if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
#     # LOG_FOUT = open(os.path.join(DUMP_DIR, 'log_evaluate.txt'), 'w')
#     # LOG_FOUT.write(str(FLAGS) + '\n')
#     ROOM_PATH_LIST = [os.path.join(ROOT_DIR, line.rstrip()) for line in open(room_data_filelist)]
#     min_x = xyz_min[0]
#     min_y = xyz_min[1]
#     min_z = xyz_min[2]
#
#     is_training = False
#     start_vect = time.time()
#     with tf.device('/gpu:' + str(GPU_INDEX)):
#         pointclouds_pl, labels_pl = placeholder_inputs(BATCH_SIZE, NUM_POINT)
#         is_training_pl = tf.placeholder(tf.bool, shape=())
#
#         # simple model
#         pred = get_model(pointclouds_pl, is_training_pl)
#         pred_softmax = tf.nn.softmax(pred)
#
#         # Add ops to save and restore all the variables.
#         saver = tf.train.Saver()
#
#     # Create a session
#     config = tf.ConfigProto()
#     config.gpu_options.allow_growth = True
#     config.allow_soft_placement = True
#     config.log_device_placement = True
#     sess = tf.Session(config=config)
#
#     # Restore variables from disk.
#     saver.restore(sess, MODEL_PATH)
#     #
#     #
#     # ops = {'pointclouds_pl': pointclouds_pl,
#     #        'labels_pl': labels_pl,
#     #        'is_training_pl': is_training_pl,
#     #        'pred_softmax': pred_softmax}
#     ops = {'pointclouds_pl': pointclouds_pl,
#            'labels_pl': labels_pl,
#            'is_training_pl': is_training_pl,
#            'pred': pred,
#            'pred_softmax': pred_softmax}
#
#     # total_correct = 0
#     # total_seen = 0
#     # fout_out_filelist = open(FLAGS.output_filelist, 'w')
#     # for room_path in ROOM_PATH_LIST:
#     room_path = ROOM_PATH_LIST[0]
#     print room_path
#
#
#
#     start_vect = time.time()
#     # error_cnt = 0
#     #
#     # total_correct = 0
#     # total_seen = 0
#     # loss_sum = 0
#     # total_seen_class = [0 for _ in range(NUM_CLASSES)]
#     # total_correct_class = [0 for _ in range(NUM_CLASSES)]
#
#
#
#     out_data_label_filename = os.path.basename(room_path)[:-4] + '_pred.txt'
#     out_data_label_filename = os.path.join(DUMP_DIR, out_data_label_filename)
#     fout_data_label = open(out_data_label_filename, 'w')
#     out_gt_label_filename = os.path.basename(room_path)[:-4] + '_gt.txt'
#     out_gt_label_filename = os.path.join(DUMP_DIR, out_gt_label_filename)
#     fout_gt_label = open(out_gt_label_filename, 'w')
#
#     out_data_ceiling_filename = os.path.basename(room_path)[:-4] + '_ceiling.ply'
#     out_data_ceiling_filename = os.path.join(DUMP_DIR, out_data_ceiling_filename)
#     cnt_ceiling = 0
#     ceiling_list = []
#     fout_ceiling_label = open(out_data_ceiling_filename, 'w')
#
#     out_data_floor_filename = os.path.basename(room_path)[:-4] + '_floor.ply'
#     out_data_floor_filename = os.path.join(DUMP_DIR, out_data_floor_filename)
#     cnt_floor = 0
#     floor_list = []
#     fout_floor_label = open(out_data_floor_filename, 'w')
#
#     out_data_wall_filename = os.path.basename(room_path)[:-4] + '_wall.ply'
#     out_data_wall_filename = os.path.join(DUMP_DIR, out_data_wall_filename)
#     cnt_wall = 0
#     wall_list = []
#     wall_list2 = []
#     fout_wall_label = open(out_data_wall_filename, 'w')
#
#     out_data_table_filename = os.path.basename(room_path)[:-4] + '_table.ply'
#     out_data_table_filename = os.path.join(DUMP_DIR, out_data_table_filename)
#     cnt_table = 0
#     table_list = []
#     fout_table_label = open(out_data_table_filename, 'w')
#
#     out_data_chair_filename = os.path.basename(room_path)[:-4] + '_chair.ply'
#     out_data_chair_filename = os.path.join(DUMP_DIR, out_data_chair_filename)
#     cnt_chair = 0
#     chair_list = []
#     chair_list2 = []
#     fout_chair_label = open(out_data_chair_filename, 'w')
#
#     # out_data_door_filename = os.path.basename(room_path)[:-4] + '_door.ply'
#     # out_data_door_filename = os.path.join(DUMP_DIR, out_data_door_filename)
#     # cnt_door = 0
#     # door_list = []
#     # fout_door_label = open(out_data_door_filename, 'w')
#
#
#
#
#     # s = datetime.datetime.now()
#     # print s
#     current_data, current_label = indoor3d_util.room2blocks_wrapper_normalized(room_path, NUM_POINT, block_size=1.5, stride=1.5)
#     current_data = current_data[:, 0:NUM_POINT, :]
#     current_label = np.squeeze(current_label)
#     # Get room dimension..
#     data_label = np.load(room_path)
#     # data_label = np.loadtxt(room_path)
#     data = data_label[:, 0:6]
#     max_room_x = max(data[:, 0])
#     max_room_y = max(data[:, 1])
#     max_room_z = max(data[:, 2])
#
#     file_size = current_data.shape[0]
#     num_batches = file_size // BATCH_SIZE
#     # print(file_size)
#
#     for batch_idx in range(num_batches):
#         start_idx = batch_idx * BATCH_SIZE
#         end_idx = (batch_idx + 1) * BATCH_SIZE
#         # cur_batch_size = end_idx - start_idx
#
#         feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
#                      ops['labels_pl']: current_label[start_idx:end_idx],
#                      ops['is_training_pl']: is_training}
#         pred_val = sess.run(ops['pred_softmax'], feed_dict=feed_dict)
#         # pred_label = np.argmax(pred_val[:, :, 0:NUM_CLASSES-1], 2)  # BxN
#         pred_label = np.argmax(pred_val, 2)  # BxN
#         # if FLAGS.no_clutter:
#         #     print FLAGS.no_clutter
#         #     pred_label = np.argmax(pred_val[:, :, 0:NUM_CLASSES-1], 2)  # BxN
#         # else:
#         #     print FLAGS.no_clutter
#         #     pred_label = np.argmax(pred_val, 2)  # BxN
#         # pred_label = np.argmax(pred_val[:, :, 0:NUM_CLASSES - 1], 2)  # BxN
#
#
#         for b in range(BATCH_SIZE):
#             pts = current_data[start_idx + b, :, :]
#             l = current_label[start_idx + b, :]
#             pts[:, 6] *= max_room_x
#             pts[:, 7] *= max_room_y
#             pts[:, 8] *= max_room_z
#             pts[:, 3:6] *= 255.0
#             pred = pred_label[b, :]
#             for i in range(NUM_POINT):
#                 # color = indoor3d_util.g_label2color[pred[i]]
#                 # color_gt = indoor3d_util.g_label2color[current_label[start_idx + b, i]]
#                 # if FLAGS.visu:
#                 #     fout.write(
#                 #         'v %f %f %f %d %d %d\n' % (pts[i, 6], pts[i, 7], pts[i, 8], color[0], color[1], color[2]))
#                 #     fout_gt.write('v %f %f %f %d %d %d\n' % (
#                 #     pts[i, 6], pts[i, 7], pts[i, 8], color_gt[0], color_gt[1], color_gt[2]))
#
#                 if pred[i] == 0:
#                     cnt_ceiling += 1
#                     coord_ceiling = str(pts[i, 6] + min_x) + ' ' + str(pts[i, 7] + min_y) + ' ' + str(pts[i, 8] + min_z) + '\n'
#                     ceiling_list.append(coord_ceiling)
#                 if pred[i] == 1:
#                     cnt_floor += 1
#                     coord_floor = str(pts[i, 6] + min_x) + ' ' + str(pts[i, 7] + min_y) + ' ' + str(pts[i, 8] + min_z) + '\n'
#                     floor_list.append(coord_floor)
#                 if pred[i] == 2:
#                     cnt_wall += 1
#                     coord_wall = str(pts[i, 6] + min_x) + ' ' + str(pts[i, 7] + min_y) + ' ' + str(pts[i, 8] + min_z) + '\n'
#                     wall_list.append(coord_wall)
#                     wall_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
#
#                 if pred[i] == 3:
#                     cnt_table += 1
#                     coord_table = str(pts[i, 6] + min_x) + ' ' + str(pts[i, 7] + min_y) + ' ' + str(pts[i, 8] + min_z) + '\n'
#                     table_list.append(coord_table)
#                 if pred[i] == 4:
#                     cnt_chair += 1
#                     coord_chair = str(pts[i, 6] + min_x) + ' ' + str(pts[i, 7] + min_y) + ' ' + str(pts[i, 8] + min_z) + '\n'
#                     chair_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
#                     chair_list.append(coord_chair)
#
#                 # if pred[i] == 5:
#                 #     cnt_door += 1
#                 #     coord_door = str(pts[i, 6] + min_x) + ' ' + str(pts[i, 7] + min_y) + ' ' + str(pts[i, 8] + min_z) + '\n'
#                 #     door_list.append(coord_door)
#                 fout_data_label.write('%f %f %f %d %d %d %f %d\n' % (
#                 pts[i, 6], pts[i, 7], pts[i, 8], pts[i, 3], pts[i, 4], pts[i, 5], pred_val[b, i, pred[i]], pred[i]))
#                 # fout_data_label.write('%f %f %f\n' % (
#                 #     pts[i, 6], pts[i, 7], pts[i, 8]))
#                 fout_gt_label.write('%d\n' % (l[i]))
#
#     fout_ceiling_label.write('ply\n'
#                           'format ascii 1.0\n'
#                           'element vertex %d\n'
#                           'property float x\n'
#                           'property float y\n'
#                           'property float z\n'
#                           'end_header\n' % cnt_ceiling)
#     fout_ceiling_label.writelines(ceiling_list)
#     fout_ceiling_label.close()
#
#     fout_floor_label.write('ply\n'
#                           'format ascii 1.0\n'
#                           'element vertex %d\n'
#                           'property float x\n'
#                           'property float y\n'
#                           'property float z\n'
#                           'end_header\n' % cnt_floor)
#     fout_floor_label.writelines(floor_list)
#     fout_floor_label.close()
#
#     fout_wall_label.write('ply\n'
#                           'format ascii 1.0\n'
#                           'element vertex %d\n'
#                           'property float x\n'
#                           'property float y\n'
#                           'property float z\n'
#                           'end_header\n' % cnt_wall)
#     fout_wall_label.writelines(wall_list)
#     fout_wall_label.close()
#
#
#
#     fout_table_label.write('ply\n'
#                           'format ascii 1.0\n'
#                           'element vertex %d\n'
#                           'property float x\n'
#                           'property float y\n'
#                           'property float z\n'
#                           'end_header\n' % cnt_table)
#     fout_table_label.writelines(table_list)
#     fout_table_label.close()
#     fout_chair_label.write('ply\n'
#                           'format ascii 1.0\n'
#                           'element vertex %d\n'
#                           'property float x\n'
#                           'property float y\n'
#                           'property float z\n'
#                           'end_header\n' % cnt_chair)
#     fout_chair_label.writelines(chair_list)
#     fout_chair_label.close()
#
#     # fout_door_label.write('ply\n'
#     #                       'format ascii 1.0\n'
#     #                       'element vertex %d\n'
#     #                       'property float x\n'
#     #                       'property float y\n'
#     #                       'property float z\n'
#     #                       'end_header\n' % cnt_door)
#     # fout_door_label.writelines(door_list)
#     # fout_door_label.close()
#
#
#     #
#     # ''' Third Process '''
#     # wall_cloud = pcl.PointCloud()
#     # wall_cloud.from_list(wall_list2)
#     # # wall_cloud = pcl.load(fout_wall_label)
#     # wall_surface_list = make_wall_info(wall_cloud, xyz_min)
#     # dae_filename = os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4])
#     # dae_filename = dae_filename+'.dae'
#     # # forth_process_runtime = (time.time() - start_vect) / 60
#     # if cnt_chair != 0:
#     #     chair_cloud = pcl.PointCloud()
#     #     chair_cloud.from_list(chair_list2)
#     #     save_path = os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4])
#     #     make_chair_info(chair_cloud, xyz_min, save_path)
#     # else:
#     #     print "There is no chairs in here!!"
#     #
#     # # print("RANSAC Process Runtime: %0.2f Minutes" % (forth_process_runtime))
#     # # start_vect = time.time()
#     # ''' Fourth Process '''
#     # poly2obj = po.Ply2Obj(wall_surface_list)
#     # poly2obj.poly_2_obj('All')
#     # poly2obj.output_dae(dae_filename)
#     # # fifth_process_runtime = (time.time() - start_vect) / 60
#     # # print("Make 3D Model Process Runtime: %0.2f Minutes" % (fifth_process_runtime))
#     # fout_data_label.close()
#     # fout_gt_label.close()
#     #
#     # print dae_filename
#     # return dae_filename
#
# if __name__ == '__main__':
#     with tf.Graph().as_default():
#         evaluate()
#
#     # LOG_FOUT.close()
#
# # import argparse
# # import os
# # import sys
# #
# # BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# # ROOT_DIR = os.path.dirname(BASE_DIR)
# # sys.path.append(BASE_DIR)
# # from model import *
# # import indoor3d_util
# #
# # parser = argparse.ArgumentParser()
# # parser.add_argument('--gpu', type=int, default=0, help='GPU to use [default: GPU 0]')
# # parser.add_argument('--batch_size', type=int, default=2, help='Batch Size during training [default: 1]')
# # parser.add_argument('--num_point', type=int, default=4096, help='Point number [default: 4096]')
# # parser.add_argument('--model_path', required=True, help='model checkpoint file path')
# # parser.add_argument('--dump_dir', required=True, help='dump folder path')
# # parser.add_argument('--output_filelist', required=True,
# #                     help='TXT filename, filelist, each line is an output for a room')
# # parser.add_argument('--room_data_filelist', required=True,
# #                     help='TXT filename, filelist, each line is a test room data label file.')
# # parser.add_argument('--no_clutter', action='store_true', help='If true, donot count the clutter class')
# # parser.add_argument('--visu', action='store_true', help='Whether to output OBJ file for prediction visualization.')
# # FLAGS = parser.parse_args()
# #
# # BATCH_SIZE = FLAGS.batch_size
# # NUM_POINT = FLAGS.num_point
# # MODEL_PATH = FLAGS.model_path
# # GPU_INDEX = FLAGS.gpu
# # DUMP_DIR = FLAGS.dump_dir
# # if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
# # LOG_FOUT = open(os.path.join(DUMP_DIR, 'log_evaluate.txt'), 'w')
# # LOG_FOUT.write(str(FLAGS) + '\n')
# # ROOM_PATH_LIST = [os.path.join(ROOT_DIR, line.rstrip()) for line in open(FLAGS.room_data_filelist)]
# #
# # NUM_CLASSES = 5
# #
# #
# # def log_string(out_str):
# #     LOG_FOUT.write(out_str + '\n')
# #     LOG_FOUT.flush()
# #     print(out_str)
# #
# #
# # def evaluate():
# #     # is_training = True
# #     is_training = False
# #
# #     with tf.device('/gpu:' + str(GPU_INDEX)):
# #         pointclouds_pl, labels_pl = placeholder_inputs(BATCH_SIZE, NUM_POINT)
# #         is_training_pl = tf.placeholder(tf.bool, shape=())
# #
# #         # simple model
# #         pred = get_model(pointclouds_pl, is_training_pl)
# #         loss = get_loss(pred, labels_pl)
# #         pred_softmax = tf.nn.softmax(pred)
# #
# #         # Add ops to save and restore all the variables.
# #         saver = tf.train.Saver()
# #
# #     # Create a session
# #     config = tf.ConfigProto()
# #     config.gpu_options.allow_growth = True
# #     config.allow_soft_placement = True
# #     config.log_device_placement = True
# #     sess = tf.Session(config=config)
# #
# #     # Restore variables from disk.
# #     saver.restore(sess, MODEL_PATH)
# #     log_string("Model restored.")
# #
# #     ops = {'pointclouds_pl': pointclouds_pl,
# #            'labels_pl': labels_pl,
# #            'is_training_pl': is_training_pl,
# #            'pred': pred,
# #            'pred_softmax': pred_softmax,
# #            'loss': loss}
# #
# #     total_correct = 0
# #     total_seen = 0
# #     fout_out_filelist = open(FLAGS.output_filelist, 'w')
# #     for room_path in ROOM_PATH_LIST:
# #         out_data_label_filename = os.path.basename(room_path)[:-4] + '_pred.txt'
# #         out_data_label_filename = os.path.join(DUMP_DIR, out_data_label_filename)
# #         out_gt_label_filename = os.path.basename(room_path)[:-4] + '_gt.txt'
# #         out_gt_label_filename = os.path.join(DUMP_DIR, out_gt_label_filename)
# #         print(room_path, out_data_label_filename)
# #         a, b = eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_gt_label_filename)
# #         total_correct += a
# #         total_seen += b
# #         fout_out_filelist.write(out_data_label_filename + '\n')
# #     fout_out_filelist.close()
# #     log_string('all room eval accuracy: %f' % (total_correct / float(total_seen)))
# #     # save_path = saver.save(sess, MODEL_PATH)
# #     # log_string("Model saved in file: %s" % save_path)
# #
# # def eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_gt_label_filename):
# #     error_cnt = 0
# #     # is_training = True
# #     is_training = False
# #     total_correct = 0
# #     total_seen = 0
# #     loss_sum = 0
# #     total_seen_class = [0 for _ in range(NUM_CLASSES)]
# #     total_correct_class = [0 for _ in range(NUM_CLASSES)]
# #     if FLAGS.visu:
# #         fout = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4] + '_pred.obj'), 'w')
# #         fout_gt = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4] + '_gt.obj'), 'w')
# #     fout_data_label = open(out_data_label_filename, 'w')
# #     fout_gt_label = open(out_gt_label_filename, 'w')
# #
# #     current_data, current_label = indoor3d_util.room2blocks_wrapper_normalized(room_path, NUM_POINT)
# #     current_data = current_data[:, 0:NUM_POINT, :]
# #     current_label = np.squeeze(current_label)
# #     # Get room dimension..
# #     data_label = np.load(room_path)
# #     data = data_label[:, 0:6]
# #     max_room_x = max(data[:, 0])
# #     max_room_y = max(data[:, 1])
# #     max_room_z = max(data[:, 2])
# #
# #     file_size = current_data.shape[0]
# #     num_batches = file_size // BATCH_SIZE
# #     print(file_size)
# #
# #     for batch_idx in range(num_batches):
# #         start_idx = batch_idx * BATCH_SIZE
# #         end_idx = (batch_idx + 1) * BATCH_SIZE
# #         cur_batch_size = end_idx - start_idx
# #
# #         feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
# #                      ops['labels_pl']: current_label[start_idx:end_idx],
# #                      ops['is_training_pl']: is_training}
# #         loss_val, pred_val = sess.run([ops['loss'], ops['pred_softmax']],
# #                                       feed_dict=feed_dict)
# #
# #         if FLAGS.no_clutter:
# #             pred_label = np.argmax(pred_val[:, :, 0:4], 2)  # BxN
# #         else:
# #             pred_label = np.argmax(pred_val, 2)  # BxN
# #         # Save prediction labels to OBJ file
# #         for b in range(BATCH_SIZE):
# #             pts = current_data[start_idx + b, :, :]
# #             l = current_label[start_idx + b, :]
# #             pts[:, 6] *= max_room_x
# #             pts[:, 7] *= max_room_y
# #             pts[:, 8] *= max_room_z
# #             pts[:, 3:6] *= 255.0
# #             pred = pred_label[b, :]
# #             for i in range(NUM_POINT):
# #                 color = indoor3d_util.g_label2color[pred[i]]
# #                 color_gt = indoor3d_util.g_label2color[current_label[start_idx + b, i]]
# #                 if FLAGS.visu:
# #                     fout.write(
# #                         'v %f %f %f %d %d %d\n' % (pts[i, 6], pts[i, 7], pts[i, 8], color[0], color[1], color[2]))
# #                     fout_gt.write('v %f %f %f %d %d %d\n' % (
# #                     pts[i, 6], pts[i, 7], pts[i, 8], color_gt[0], color_gt[1], color_gt[2]))
# #                 fout_data_label.write('%f %f %f %d %d %d %f %d\n' % (
# #                 pts[i, 6], pts[i, 7], pts[i, 8], pts[i, 3], pts[i, 4], pts[i, 5], pred_val[b, i, pred[i]], pred[i]))
# #                 fout_gt_label.write('%d\n' % (l[i]))
# #         correct = np.sum(pred_label == current_label[start_idx:end_idx, :])
# #         total_correct += correct
# #         total_seen += (cur_batch_size * NUM_POINT)
# #         loss_sum += (loss_val * BATCH_SIZE)
# #         for i in range(start_idx, end_idx):
# #             for j in range(NUM_POINT):
# #                 l = current_label[i, j]
# #                 total_seen_class[l] += 1
# #                 total_correct_class[l] += (pred_label[i - start_idx, j] == l)
# #
# #
# #
# #
# #     log_string('eval mean loss: %f' % (loss_sum / float(total_seen / NUM_POINT)))
# #     log_string('eval accuracy: %f' % (total_correct / float(total_seen)))
# #     fout_data_label.close()
# #     fout_gt_label.close()
# #     if FLAGS.visu:
# #         fout.close()
# #         fout_gt.close()
# #     return total_correct, total_seen
# #
# #
# # if __name__ == '__main__':
# #     with tf.Graph().as_default():
# #         evaluate()
# #     LOG_FOUT.close()