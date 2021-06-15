# -*- coding: utf-8 -*-
"""
Input:Las -> H5 -> PointNet -> Output:H5,Las

@author: jessieyen
"""
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
import model
import indoor3d_util
import numpy as np
import tensorflow as tf
import logging
import pcl
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter('%(asctime)s %(funcName)s [%(levelname)s]: %(message)s'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.addHandler(handler)

# default
BATCH_SIZE = 1
NUM_POINT = 4096
MODEL_PATH = ''
GPU_INDEX = 0
DATA_PATH = None

NUM_CLASSES = 6

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)

def evaluate(model_path, out_filename, booth_data):
    global MODEL_PATH
    MODEL_PATH = model_path
    DUMP_DIR = os.path.join(os.path.dirname(out_filename), 'dump')
    if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
    with tf.device('/gpu:' + str(GPU_INDEX)):
        pointclouds_pl, labels_pl = model.placeholder_inputs(BATCH_SIZE, NUM_POINT)
        is_training_pl = tf.placeholder(tf.bool, shape=())

        # simple model
        pred = model.get_model(pointclouds_pl, is_training_pl)
        loss = model.get_loss(pred, labels_pl)
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
    # print(MODEL_PATH)
    saver.restore(sess, MODEL_PATH)

    ops = {'pointclouds_pl': pointclouds_pl,
           'labels_pl': labels_pl,
           'is_training_pl': is_training_pl,
           'pred': pred,
           'pred_softmax': pred_softmax,
           'loss': loss}

    # Output Filename


    all_ceiling, all_floor, all_wall, all_window, all_door, all_clutter = eval_one_epoch(sess, ops, booth_data)
    logger.info("Result of Ceiling : " + str(len(all_ceiling)))
    logger.info("Result of Floor : " + str(len(all_floor)))
    logger.info("Result of Wall : " + str(len(all_wall)))
    logger.info("Result of Window : " + str(len(all_window)))
    logger.info("Result of Door : " + str(len(all_door)))
    logger.info("Result of Clutter : " + str(len(all_clutter)))
    logger.info("Finishing the PointNet")
    wall_cloud = pcl.PointCloud()
    wall_cloud.from_list(all_wall)
    door_cloud = pcl.PointCloud()
    door_cloud.from_list(all_door)
    window_cloud = pcl.PointCloud()
    window_cloud.from_list(all_window)

    pcl.save(wall_cloud,
             os.path.join(DUMP_DIR, os.path.basename(out_filename)) + '_wall' + ".pcd")
    return wall_cloud, door_cloud, window_cloud
def eval_one_epoch(sess, ops, booth_data):
    is_training = False

    # Load Data
    data = booth_data
    min_room_x = min(data[:, 0])
    min_room_y = min(data[:, 1])
    min_room_z = min(data[:, 2])
    data[:, 0] -= min_room_x
    data[:, 1] -= min_room_y
    data[:, 2] -= min_room_z
    max_room_x = max(data[:, 0])
    max_room_y = max(data[:, 1])
    max_room_z = max(data[:, 2])

    current_data, current_label = indoor3d_util.room2blocks_plus_normalized(data, NUM_POINT, block_size=1.0, stride=1.0,
                                                                            random_sample=False, sample_num=None,
                                                                            sample_aug=1)
    current_data = current_data[:, 0:NUM_POINT, :]
    current_label = np.squeeze(current_label)
    file_size = current_data.shape[0]
    num_batches = file_size // BATCH_SIZE
    print(file_size)
    output_data = []
    output_label = []
    output_name = []
    for batch_idx in range(num_batches):
        start_idx = batch_idx * BATCH_SIZE
        end_idx = (batch_idx + 1) * BATCH_SIZE

        feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                     ops['labels_pl']: current_label[start_idx:end_idx],
                     ops['is_training_pl']: is_training}
        loss_val, pred_val = sess.run([ops['loss'], ops['pred_softmax']],
                                      feed_dict=feed_dict)

        pred_label = np.argmax(pred_val, 2)  # BxN

        output_label.extend(list(np.reshape(pred_label, NUM_POINT)))

    output_data = np.reshape(current_data, (file_size * NUM_POINT, 9))

    output_data[:, 0] = (output_data[:, 6] * max_room_x) + min_room_x
    output_data[:, 1] = (output_data[:, 7] * max_room_y) + min_room_y
    output_data[:, 2] = (output_data[:, 8] * max_room_z) + min_room_z

    output_data[:, 3:6] *= 65535.0
    output_label = np.asarray(output_label)

    label2name = {0: 'ceiling',
                  1: 'floor',
                  2: 'wall',
                  3: 'window',
                  4: 'door',
                  5: 'clutter'}

    output_name = np.empty((file_size * NUM_POINT, 2), dtype='object')
    all_ceiling = list()
    all_floor = list()
    all_wall = list()
    all_window = list()
    all_door = list()
    all_clutter = list()
    print len(output_label), len(output_data)
    for label_i in range(len(output_label)):
        label_info = output_label[label_i]
        xyz_data = output_data[label_i][:3]
        if label_info == 0:
            all_ceiling.append(xyz_data)
        elif label_info == 1:
            all_floor.append(xyz_data)
        elif label_info == 2:
            all_wall.append(xyz_data)
        elif label_info == 3:
            all_window.append(xyz_data)
            all_wall.append(xyz_data)
        elif label_info == 4:
            all_door.append(xyz_data)
            all_wall.append(xyz_data)
        elif label_info == 5:
            all_clutter.append(xyz_data)


    return  all_ceiling, all_floor, all_wall, all_window, all_door, all_clutter
    # for key, values in label2name.items():
    #     print output_label, key, values
    #     output_name[output_label == key, 0] = values
    #
    # print("result output label")
    # print(output_data)
    # #
    # data_util.save_h5(out_data_label_filename + '.h5', output_data, output_label, output_name)
    # data_util.save_las(out_data_label_filename + '.las', output_data, output_label)



def setArgs(**kwargs):  # batch_size,num_point,model_path,gpu,data_path
    global BATCH_SIZE
    global NUM_POINT
    global MODEL_PATH
    global GPU_INDEX
    global DATA_PATH

    try:
        BATCH_SIZE = kwargs["batch_size"]
    except KeyError:
        pass
    try:
        NUM_POINT = kwargs["num_point"]
    except KeyError:
        pass
    try:
        MODEL_PATH = kwargs["model_path"]
    except KeyError:
        pass
    try:
        GPU_INDEX = kwargs["gpu"]
    except KeyError:
        pass
    try:
        DATA_PATH = kwargs["data_path"]
    except KeyError:
        pass




# if __name__ == '__main__':
#     with tf.Graph().as_default():
#         evaluate()
