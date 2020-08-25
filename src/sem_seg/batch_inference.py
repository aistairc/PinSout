# -*- coding:utf-8 -*-
import argparse
import os
import sys
import pcl as pc
from collections import OrderedDict

from src.plane_ransac import Make_CityGML_Data as mcd2
from src.citygml import PointCloud_To_CityGML as gml2

file_data = OrderedDict()
file_wall_data = OrderedDict()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)

from model import *
import indoor3d_util

# from IPython.core.debugger import Tracer

parser = argparse.ArgumentParser()
parser.add_argument('--gpu', type=int, default=0, help='GPU to use [default: GPU 0]')
parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during training [default: 1]')
parser.add_argument('--num_point', type=int, default=4096, help='Point number [default: 4096]')
parser.add_argument('--model_path', required=True, help='model checkpoint file path')
parser.add_argument('--dump_dir', required=True, help='dump folder path')
parser.add_argument('--output_filelist', required=True,
                    help='TXT filename, filelist, each line is an output for a room')
parser.add_argument('--room_data_filelist', required=True,
                    help='TXT filename, filelist, each line is a test room data label file.')
parser.add_argument('--no_clutter', action='store_true', help='If true, donot count the clutter class')
parser.add_argument('--visu', action='store_true', help='Whether to output OBJ file for prediction visualization.')
FLAGS = parser.parse_args()

BATCH_SIZE = FLAGS.batch_size
NUM_POINT = FLAGS.num_point
MODEL_PATH = FLAGS.model_path
GPU_INDEX = FLAGS.gpu
DUMP_DIR = FLAGS.dump_dir
if not os.path.exists(DUMP_DIR):
    os.mkdir(DUMP_DIR)
LOG_FOUT = open(os.path.join(DUMP_DIR, 'log_evaluate.txt'), 'w')
LOG_FOUT.write(str(FLAGS) + '\n')
ROOM_PATH_LIST = [os.path.join(ROOT_DIR, line.rstrip()) for line in open(FLAGS.room_data_filelist)]

# NUM_CLASSES = 6
# NUM_CLASSES = 13
NUM_CLASSES = 5
upsilon = 0.05

def log_string(out_str):
    LOG_FOUT.write(out_str + '\n')
    LOG_FOUT.flush()
    print(out_str)


def evaluate():
    is_training = False

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
    log_string("Model restored.")

    ops = {'pointclouds_pl': pointclouds_pl,
           'labels_pl': labels_pl,
           'is_training_pl': is_training_pl,
           'pred': pred,
           'pred_softmax': pred_softmax,
           'loss': loss}

    total_correct = 0
    total_seen = 0
    fout_out_filelist = open(FLAGS.output_filelist, 'w')
    for room_path in ROOM_PATH_LIST:
        out_data_label_filename = os.path.basename(room_path)[:-4] + '_pred.ply'
        out_data_label_filename = os.path.join(DUMP_DIR, out_data_label_filename)
        out_data_ceiling_filename = os.path.basename(room_path)[:-4] + '_ceiling.ply'
        out_data_ceiling_filename = os.path.join(DUMP_DIR, out_data_ceiling_filename)
        out_data_floor_filename = os.path.basename(room_path)[:-4] + '_floor.ply'
        out_data_floor_filename = os.path.join(DUMP_DIR, out_data_floor_filename)
        out_data_wall_filename = os.path.basename(room_path)[:-4] + '_wall.ply'
        out_data_wall_filename = os.path.join(DUMP_DIR, out_data_wall_filename)
        out_data_window_filename = os.path.basename(room_path)[:-4] + '_window.ply'
        out_data_window_filename = os.path.join(DUMP_DIR, out_data_window_filename)
        out_data_door_filename = os.path.basename(room_path)[:-4] + '_door.ply'
        out_data_door_filename = os.path.join(DUMP_DIR, out_data_door_filename)
        out_gt_label_filename = os.path.basename(room_path)[:-4] + '_gt.ply'
        out_gt_label_filename = os.path.join(DUMP_DIR, out_gt_label_filename)
        print(room_path, out_data_label_filename)
        a, b = eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_data_ceiling_filename,
                              out_data_floor_filename, out_data_wall_filename, out_data_window_filename,
                              out_data_door_filename, out_gt_label_filename)
        total_correct += a
        total_seen += b
        fout_out_filelist.write(out_data_label_filename + '\n')
    fout_out_filelist.close()
    log_string('all room eval accuracy: %f' % (total_correct / float(total_seen)))


def eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_data_ceiling_filename,
                   out_data_floor_filename, out_data_wall_filename, out_data_window_filename,
                   out_data_door_filename, out_gt_label_filename):
    error_cnt = 0
    is_training = False
    total_correct = 0
    total_seen = 0
    loss_sum = 0
    cnt_data = 0
    cnt_ceiling = 0
    cnt_floor = 0
    cnt_wall = 0
    cnt_window = 0
    cnt_door = 0
    max_list = []
    min_list = []
    data_list = []
    ceiling_list = []
    floor_list = []
    wall_list = []
    window_list = []
    door_list = []

    data_list2 = []
    ceiling_list2 = []
    floor_list2 = []
    wall_list2 = []
    window_list2 = []
    door_list2 = []

    total_seen_class = [0 for _ in range(NUM_CLASSES)]
    total_correct_class = [0 for _ in range(NUM_CLASSES)]
    if FLAGS.visu:
        fout = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4] + '_pred.obj'), 'w')
        fout_gt = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4] + '_gt.obj'), 'w')
    fout_data_label = open(out_data_label_filename, 'w')
    fout_gt_label = open(out_gt_label_filename, 'w')
    fout_ceiling_label = open(out_data_ceiling_filename, 'w')
    fout_floor_label = open(out_data_floor_filename, 'w')
    fout_wall_label = open(out_data_wall_filename, 'w')
    fout_window_label = open(out_data_window_filename, 'w')
    fout_door_label = open(out_data_door_filename, 'w')

    current_data, current_label = indoor3d_util.room2blocks_wrapper_normalized(room_path, NUM_POINT)
    current_data = current_data[:, 0:NUM_POINT, :]
    current_label = np.squeeze(current_label)
    # Get room dimension..
    data_label = np.load(room_path)
    data = data_label[:, 0:6]
    max_room_x = max(data[:, 0])
    max_room_y = max(data[:, 1])
    max_room_z = max(data[:, 2])

    file_size = current_data.shape[0]
    num_batches = file_size // BATCH_SIZE  # 31 = 31 // 1

    for batch_idx in range(num_batches):
        # print('ERROR')
        start_idx = batch_idx * BATCH_SIZE
        end_idx = (batch_idx + 1) * BATCH_SIZE
        cur_batch_size = end_idx - start_idx
        # print('ERROR')
        feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                     ops['labels_pl']: current_label[start_idx:end_idx],
                     ops['is_training_pl']: is_training}
        loss_val, pred_val = sess.run([ops['loss'], ops['pred_softmax']],
                                      feed_dict=feed_dict)

        if FLAGS.no_clutter:
            # pred_label = np.argmax(pred_val[:, :, 0:12], 2)  # BxN
            pred_label = np.argmax(pred_val[:, :, 0:12], 2)  # 5개의 클래스
        else:
            pred_label = np.argmax(pred_val, 2)  # BxN
        # Save prediction labels to OBJ file
        for b in range(BATCH_SIZE):

            pts = current_data[start_idx + b, :, :]
            l = current_label[start_idx + b, :]
            pts[:, 6] *= max_room_x
            pts[:, 7] *= max_room_y
            pts[:, 8] *= max_room_z
            pts[:, 3:6] *= 255.0
            pred = pred_label[b, :]
            for i in range(NUM_POINT):
                color = indoor3d_util.g_label2color[pred[i]]
                color_gt = indoor3d_util.g_label2color[current_label[start_idx + b, i]]
                if FLAGS.visu:
                    fout.write('v %f %f %f %d %d %d\n' %
                               (pts[i, 6], pts[i, 7], pts[i, 8], color[0], color[1], color[2]))
                    fout_gt.write('v %f %f %f %d %d %d\n' % (pts[i, 6], pts[i, 7], pts[i, 8], color_gt[0], color_gt[1],
                                                             color_gt[2]))
                if pred[i] == 0:
                    cnt_ceiling += 1
                    coord_ceiling = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                    ceiling_list.append(coord_ceiling)
                    ceiling_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
            
                if pred[i] == 1:
                    cnt_floor += 1
                    coord_floor = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                    floor_list.append(coord_floor)
                    floor_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                   
                if pred[i] == 2:
                    
                    cnt_wall += 1
                    coord_wall = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                    wall_list.append(coord_wall)
                    wall_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                
                if pred[i] == 3:
                    cnt_window += 1
                    coord_window = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                    window_list.append(coord_window)
                    window_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                  
                if pred[i] == 4:
                    cnt_door += 1
                    coord_door = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                    door_list.append(coord_door)
                    door_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
           
                coord_data = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                data_list.append(coord_data)
                data_list2.append([pts[i, 6], pts[i, 7], pts[i, 8]])
                cnt_data += 1
                fout_gt_label.write('%d\n' % (l[i]))

        correct = np.sum(pred_label == current_label[start_idx:end_idx, :])
        total_correct += correct
        total_seen += (cur_batch_size * NUM_POINT)
        loss_sum += (loss_val * BATCH_SIZE)
        for i in range(start_idx, end_idx):
            for j in range(NUM_POINT):
                l = current_label[i, j]
                total_seen_class[l] += 1
                total_correct_class[l] += (pred_label[i - start_idx, j] == l)

    fout_data_label.write('ply\n'
                          'format ascii 1.0\n'
                          'element vertex %d\n'
                          'property float x\n'
                          'property float y\n'
                          'property float z\n'
                          'end_header\n' % cnt_data)
    fout_data_label.writelines(data_list)
    fout_data_label.close()
    fout_ceiling_label.write('ply\n'
                             'format ascii 1.0\n'
                             'element vertex %d\n'
                             'property float x\n'
                             'property float y\n'
                             'property float z\n'
                             'end_header\n' % cnt_ceiling)
    fout_ceiling_label.writelines(ceiling_list)
    fout_ceiling_label.close()
    fout_floor_label.write('ply\n'
                           'format ascii 1.0\n'
                           'element vertex %d\n'
                           'property float x\n'
                           'property float y\n'
                           'property float z\n'
                           'end_header\n' % cnt_floor)
    fout_floor_label.writelines(floor_list)
    fout_floor_label.close()
    fout_wall_label.write('ply\n'
                           'format ascii 1.0\n'
                           'element vertex %d\n'
                           'property float x\n'
                           'property float y\n'
                           'property float z\n'
                           'end_header\n' % cnt_wall)
    fout_wall_label.writelines(wall_list)
    fout_wall_label.close()
    fout_window_label.write('ply\n'
                           'format ascii 1.0\n'
                           'element vertex %d\n'
                           'property float x\n'
                           'property float y\n'
                           'property float z\n'
                           'end_header\n' % cnt_window)
    fout_window_label.writelines(window_list)
    fout_window_label.close()
    fout_door_label.write('ply\n'
                           'format ascii 1.0\n'
                           'element vertex %d\n'
                           'property float x\n'
                           'property float y\n'
                           'property float z\n'
                           'end_header\n' % cnt_door)
    fout_door_label.writelines(door_list)
    fout_door_label.close()
    pred_cloud = pc.PointCloud()
    ceiling_cloud = pc.PointCloud()
    floor_cloud = pc.PointCloud()
    wall_cloud = pc.PointCloud()
    door_cloud = pc.PointCloud()
    window_cloud = pc.PointCloud()

    pred_cloud.from_array(np.asarray(data_list2, dtype=np.float32))
    ceiling_cloud.from_array(np.asarray(ceiling_list2, dtype=np.float32))
    floor_cloud.from_array(np.asarray(floor_list2, dtype=np.float32))
    wall_cloud.from_array(np.asarray(wall_list2, dtype=np.float32))
    if len(door_list2) != 0:
        door_cloud.from_array(np.asarray(door_list2, dtype=np.float32))
    if len(window_list2) != 0:
        window_cloud.from_array(np.asarray(window_list2, dtype=np.float32))

    make_gml_data2 = mcd2.MakeCityGMLData2(pred_cloud, ceiling_cloud, floor_cloud, wall_cloud, door_cloud, window_cloud)

    wall_surface, ceiling_surface, floor_surface, door_surface, window_surface = make_gml_data2.make_point_surface()
    make_gml_file2 = gml2.PointCloudToCityGML2(ceiling_surface, floor_surface, wall_surface, door_surface, window_surface)
    make_gml_file2.MakeRoomObject()

    log_string('eval mean loss: %f' % (loss_sum / float(total_seen / NUM_POINT)))
    log_string('eval accuracy: %f' % (total_correct / float(total_seen)))
    fout_data_label.close()
    fout_gt_label.close()



    if FLAGS.visu:
        fout.close()
        fout_gt.close()
    return total_correct, total_seen




if __name__ == '__main__':
    with tf.Graph().as_default():
        evaluate()
    LOG_FOUT.close()
