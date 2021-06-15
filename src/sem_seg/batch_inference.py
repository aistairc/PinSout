
import os
import sys
import pcl
import numpy as np
import tensorflow as tf
from model import *
import indoor3d_util
import sys
import logging
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)
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
    :param min_list: min xyz of each npy data
    :return: 3D model path, bounding box area of chair data [max point(x, y, z), min point(x, y, z]
    '''
    is_training = False

    MODEL_PATH = model_path

    DUMP_DIR = os.path.join(os.path.dirname(out_filename), 'dump')
    if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
    # ROOM_PATH_LIST = [os.path.join(ROOT_DIR, line.rstrip()) for line in open(room_data_filelist)]

    with tf.device('/gpu:' + str(GPU_INDEX)):
        pointclouds_pl, labels_pl = placeholder_inputs(BATCH_SIZE, NUM_POINT)
        is_training_pl = tf.compat.v1.placeholder(tf.bool, shape=())

        # simple model
        pred = get_model(pointclouds_pl, is_training_pl)
        loss = get_loss(pred, labels_pl)
        pred_softmax = tf.nn.softmax(pred)

        # Add ops to save and restore all the variables.
        saver = tf.compat.v1.train.Saver()

    # Create a session
    config = tf.compat.v1.ConfigProto()
    config.gpu_options.allow_growth = True
    config.allow_soft_placement = True
    config.log_device_placement = True
    sess = tf.compat.v1.Session(config=config)

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
    all_window = list()
    all_door = list()
    all_clutter = list()


    logger.info("Starting the PointNet")
    for each_i in range(len(booth_data)):

        ceiling_list, floor_list, wall_list, window_list, door_list, clutter_list = eval_one_epoch(sess, ops, booth_data[each_i])
        if len(ceiling_list) != 0:
            temp_value_1 = np.asarray(ceiling_list)
            temp_value_1 += min_list[each_i]
            ceiling_cloud = pcl.PointCloud()
            ceiling_cloud.from_list(temp_value_1.tolist())
            all_ceiling += temp_value_1.tolist()
            pcl.save(ceiling_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_ceiling' + "_" + str(each_i)+".pcd")
        if len(floor_list) != 0:
            temp_value_2 = np.asarray(floor_list)
            temp_value_2 += min_list[each_i]
            floor_cloud = pcl.PointCloud()
            floor_cloud.from_list(temp_value_2.tolist())
            pcl.save(floor_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_floor' + "_" + str(each_i)+".pcd")
            all_floor += temp_value_2.tolist()
        if len(wall_list) != 0:
            temp_value_3 = np.asarray(wall_list)
            temp_value_3 += min_list[each_i]
            wall_cloud = pcl.PointCloud()
            wall_cloud.from_list(temp_value_3.tolist())
            pcl.save(wall_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_wall' + "_" + str(each_i) + ".ply")
            all_wall += temp_value_3.tolist()
        if len(window_list) != 0:
            temp_value_4 = np.asarray(window_list)
            temp_value_4 += min_list[each_i]
            window_cloud = pcl.PointCloud()
            window_cloud.from_list(temp_value_4.tolist())
            pcl.save(window_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) +'_window' + "_" + str(each_i)+".pcd")
            all_window += temp_value_4.tolist()
            # all_wall += temp_value_4.tolist()
        if len(door_list) != 0:
            temp_value_5 = np.asarray(door_list)
            temp_value_5 += min_list[each_i]
            door_cloud = pcl.PointCloud()
            door_cloud.from_list(temp_value_5.tolist())
            pcl.save(door_cloud, os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_door' + "_" + str(each_i)+".pcd")
            all_door += temp_value_5.tolist()
            # all_wall += temp_value_5.tolist()
        if len(clutter_list) != 0:
            temp_value_6 = np.asarray(clutter_list)
            temp_value_6 += min_list[each_i]
            clutter_cloud = pcl.PointCloud()
            clutter_cloud.from_list(temp_value_6.tolist())
            all_clutter += temp_value_6.tolist()
            pcl.save(clutter_cloud,
                     os.path.join(DUMP_DIR, os.path.basename(out_filename)) + "_clutter" + "_" + str(each_i)+".pcd")

    ''' Third Process '''

    logger.info("Result of Ceiling : " + str(len(all_ceiling)))
    logger.info("Result of Floor : " + str(len(all_floor)))
    logger.info("Result of Wall : "+ str(len(all_wall)))
    logger.info("Result of Window : "+str(len(all_window)))
    logger.info("Result of Door : "+str(len(all_door)))
    logger.info("Result of Clutter : "+str(len(all_clutter)))
    logger.info("Finishing the PointNet")

    wall_cloud = pcl.PointCloud()
    wall_cloud.from_list(all_wall + all_door + all_window)
    door_cloud = pcl.PointCloud()
    door_cloud.from_list(all_door)
    window_cloud = pcl.PointCloud()
    window_cloud.from_list(all_window)

    pcl.save(wall_cloud,
             os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_wall' + ".ply")
    return wall_cloud, door_cloud, window_cloud


def eval_one_epoch(sess, ops, each_data):

    is_training = False
    ceiling_list = list()
    floor_list = list()
    wall_list = list()
    window_list = list()
    door_list = list()
    clutter_list = list()


    current_data, current_label = indoor3d_util.room2blocks_wrapper_normalized(each_data, NUM_POINT)
    current_data = current_data[:, 0:NUM_POINT, :]
    current_label = np.squeeze(current_label)
    # Get room dimension..
    data_label = each_data
    data = data_label[:, 0:6]
    max_room_x = max(data[:, 0])
    max_room_y = max(data[:, 1])
    max_room_z = max(data[:, 2])
    file_size = current_data.shape[0]

    num_batches = file_size // BATCH_SIZE


    for batch_idx in range(num_batches):
        start_idx = batch_idx * BATCH_SIZE
        end_idx = (batch_idx + 1) * BATCH_SIZE
        feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                     ops['labels_pl']: current_label[start_idx:end_idx],
                     ops['is_training_pl']: is_training}

        pred_val = sess.run(ops['pred_softmax'], feed_dict=feed_dict)

        pred_label = np.argmax(pred_val, 2)

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
                    window_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                if pred[i] == 4:
                    door_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                if pred[i] == 5:
                    clutter_list.append([pts[i, 6], pts[i, 7], pts[i, 8]])

    return ceiling_list, floor_list, wall_list, window_list, door_list, clutter_list


