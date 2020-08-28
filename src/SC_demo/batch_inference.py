#-*- coding:utf-8 -*-
import argparse
import os
import sys
import csv
import json
import numpy as np
import tensorflow as tf
import pcl.boundaries as bn
import pcl as pc
from collections import OrderedDict
import checkArguments as check
from visvalingam import VisvalingamSimplification
file_data = OrderedDict()

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
parser.add_argument('--output_filelist', required=True, help='TXT filename, filelist, each line is an output for a room')
parser.add_argument('--room_data_filelist', required=True, help='TXT filename, filelist, each line is a test room data label file.')
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
LOG_FOUT.write(str(FLAGS)+'\n')
ROOM_PATH_LIST = [os.path.join(ROOT_DIR, line.rstrip()) for line in open(FLAGS.room_data_filelist)]

# NUM_CLASSES = 6
# NUM_CLASSES = 13
NUM_CLASSES = 5

def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    print(out_str)

def evaluate():
    is_training = False
     
    with tf.device('/gpu:'+str(GPU_INDEX)):
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
        out_data_label_filename = os.path.basename(room_path)[:-4] + '_pred.txt'
        out_data_label_filename = os.path.join(DUMP_DIR, out_data_label_filename)
        out_data_ceiling_filename = os.path.basename(room_path)[:-4] + '_ceiling.ply'
        out_data_ceiling_filename = os.path.join(DUMP_DIR, out_data_ceiling_filename)
        out_data_floor_filename = os.path.basename(room_path)[:-4] + '_floor.txt'
        out_data_floor_filename = os.path.join(DUMP_DIR, out_data_floor_filename)
        out_data_wall_filename = os.path.basename(room_path)[:-4] + '_wall.txt'
        out_data_wall_filename = os.path.join(DUMP_DIR, out_data_wall_filename)
        out_data_window_filename = os.path.basename(room_path)[:-4] + '_window.txt'
        out_data_window_filename = os.path.join(DUMP_DIR, out_data_window_filename)
        out_data_door_filename = os.path.basename(room_path)[:-4] + '_door.txt'
        out_data_door_filename = os.path.join(DUMP_DIR, out_data_door_filename)
        out_gt_label_filename = os.path.basename(room_path)[:-4] + '_gt.txt'
        out_gt_label_filename = os.path.join(DUMP_DIR, out_gt_label_filename)
        print(room_path, out_data_label_filename)
        a, b = eval_one_epoch(sess, ops, room_path, out_data_label_filename, out_data_ceiling_filename,
                              out_data_floor_filename, out_data_wall_filename, out_data_window_filename,
                              out_data_door_filename, out_gt_label_filename, )
        total_correct += a
        total_seen += b
        fout_out_filelist.write(out_data_label_filename+'\n')
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
    cnt_ceiling = 0
    cnt_floor = 0
    max_list = []
    min_list = []
    ceiling_list = []
    floor_list = []
    total_seen_class = [0 for _ in range(NUM_CLASSES)]
    total_correct_class = [0 for _ in range(NUM_CLASSES)]
    if FLAGS.visu:
        fout = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4]+'_pred.obj'), 'w')
        fout_gt = open(os.path.join(DUMP_DIR, os.path.basename(room_path)[:-4]+'_gt.obj'), 'w')
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
    num_batches = file_size // BATCH_SIZE    # 31 = 31 // 1
    
    for batch_idx in range(num_batches):
        # print('ERROR')
        start_idx = batch_idx * BATCH_SIZE
        end_idx = (batch_idx+1) * BATCH_SIZE
        cur_batch_size = end_idx - start_idx
        # print('ERROR')
        feed_dict = {ops['pointclouds_pl']: current_data[start_idx:end_idx, :, :],
                     ops['labels_pl']: current_label[start_idx:end_idx],
                     ops['is_training_pl']: is_training}
        loss_val, pred_val = sess.run([ops['loss'], ops['pred_softmax']],
                                      feed_dict=feed_dict)

        if FLAGS.no_clutter:
            # pred_label = np.argmax(pred_val[:, :, 0:12], 2)  # BxN
            pred_label = np.argmax(pred_val[:, :, 0:12], 2)   # 5개의 클래스
        else:
            pred_label = np.argmax(pred_val, 2)  # BxN
        # Save prediction labels to OBJ file
        for b in range(BATCH_SIZE):

            pts = current_data[start_idx+b, :, :]
            l = current_label[start_idx+b, :]
            pts[:, 6] *= max_room_x
            pts[:, 7] *= max_room_y
            pts[:, 8] *= max_room_z
            pts[:, 3:6] *= 255.0
            pred = pred_label[b, :]
            for i in range(NUM_POINT):
                color = indoor3d_util.g_label2color[pred[i]]
                color_gt = indoor3d_util.g_label2color[current_label[start_idx+b, i]]
                if FLAGS.visu:
                    fout.write('v %f %f %f %d %d %d\n' %
                               (pts[i, 6], pts[i, 7], pts[i, 8], color[0], color[1], color[2]))
                    fout_gt.write('v %f %f %f %d %d %d\n' % (pts[i, 6], pts[i, 7], pts[i, 8], color_gt[0], color_gt[1],
                                                             color_gt[2]))
                    fout_data_label.write('%f,%f,%f\n' % (pts[i, 6], pts[i, 7], pts[i, 8]))
                if pred[i] == 0:
                    cnt_ceiling += 1
                    coord_ceiling = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + '\n'
                    ceiling_list.append(coord_ceiling)
                    max_z = pts[i, 8]
                    max_list.append(max_z)
                    max_value = np.max(max_list)
                if pred[i] == 1:
                    cnt_floor += 1
                    coord_floor = str(pts[i, 6]) + ' ' + str(pts[i, 7]) + ' ' + str(pts[i, 8]) + ' ' +\
                                  str(pts[i, 3]) + ' ' + str(pts[i, 4]) + ' ' + str(pts[i, 5]) + '\n'
                    floor_list.append(coord_floor)
                    min_z = pts[i, 8]
                    min_list.append(min_z)
                    min_value = np.min(min_list)
                if pred[i] == 2:
                    fout_wall_label.write('%f,%f,%f\n' % (pts[i, 6], pts[i, 7], pts[i, 8]))
                if pred[i] == 3:
                    fout_window_label.write('%f %f %f %d %d %d\n' % (pts[i, 6], pts[i, 7], pts[i, 8], pts[i, 3],
                                                                     pts[i, 4], pts[i, 5]))
                if pred[i] == 4:
                    fout_door_label.write('%f %f %f %d %d %d\n' % (pts[i, 6], pts[i, 7], pts[i, 8], pts[i, 3],
                                                                   pts[i, 4], pts[i, 5]))
                fout_gt_label.write('%d\n' % (l[i]))
        correct = np.sum(pred_label == current_label[start_idx:end_idx, :])
        total_correct += correct
        total_seen += (cur_batch_size*NUM_POINT)
        loss_sum += (loss_val*BATCH_SIZE)
        for i in range(start_idx, end_idx):
            for j in range(NUM_POINT):
                l = current_label[i, j]
                total_seen_class[l] += 1
                total_correct_class[l] += (pred_label[i-start_idx, j] == l)
    fout_ceiling_label.write('ply\n'
                             'format ascii 1.0\n'
                             'element vertex %d\n'
                             'property float x\n'
                             'property float y\n'
                             'property float z\n'
                             'end_header\n' % cnt_ceiling)
    fout_ceiling_label.writelines(ceiling_list)
    fout_ceiling_label.close()
    """fout_floor_label.write('ply\n'
                           'format ascii 1.0\n'
                           'element vertex %d\n'
                           'property float x\n'
                           'property float y\n'
                           'property float z\n'
                           'property uchar red\n'
                           'property uchar blue\n'
                           'property uchar green\n'
                           'end_header\n' % cnt_floor)
    fout_floor_label.writelines(floor_list)
    fout_floor_label.close()"""
    log_string('eval mean loss: %f' % (loss_sum / float(total_seen/NUM_POINT)))
    log_string('eval accuracy: %f' % (total_correct / float(total_seen)))
    fout_data_label.close()
    fout_gt_label.close()
    cloud = pc.load("/home/dprt/pointnet/sem_seg/log_5cls/zz/Area_4_office_4_ceiling.ply")
    data = np.asarray(cloud)
    boundary = bn.estimate_boundaries(cloud, 1.0, 1.0, 0, 0.1)
    boundary_list = data[boundary, :]
    file_data["type"] = "LineString"
    file_data["coordinates"] = []
    with open('/home/dprt/pointnet/sem_seg/log_5cls/zz/test_merge.csv', 'w') as csv_file:
        writer = csv.writer(csv_file)
        for i in boundary_list:
            matrix = []
            writer.writerow([i[0], i[1], i[2]])
            matrix.append(float(i[0]))
            matrix.append(float(i[1]))
            file_data["coordinates"].append(matrix)
    # with open('/home/dprt/pointnet/sem_seg/log_5cls/zz/convert_json.json', 'w') as make_file:
    #     json.dump(file_data, make_file)

    threshold = 0.1
    nPointsPrev = len(file_data['coordinates'])

    simplify = VisvalingamSimplification(file_data['coordinates'])
    file_data['coordinates'] = simplify.simplifyLineString(float(threshold))
    file_data['threshold'] = threshold

    print('Pointreduction:', nPointsPrev, '/', len(file_data['coordinates']))

    # write the resulting GeoJSON-file
    json_file = open('/home/dprt/pointnet/sem_seg/log_5cls/zz/convert_json_sim.json', 'w')
    json.dump(file_data, json_file)
    # json_file.close()
    # with open('/home/dprt/pointnet/sem_seg/log_5cls/zz/convert_json_sim.json', 'r') as json_file_2:
    #     json_data = json_file_2.read()
    # jdata = json.load(json_data)

    with open('/home/dprt/pointnet/sem_seg/log_5cls/zz/end.csv', 'w') as csv_file2:
        writer = csv.writer(csv_file2)
        value = file_data["coordinates"]
        for i in value:
            matrix2 = []
            matrix2.append(float(i[0]))
            matrix2.append(float(i[1]))
            matrix2.append(max_value)
            writer.writerow(matrix2)

    h = max_value - min_value     # Height
    print(h)
    print(max_value)
    print(min_value)

    if FLAGS.visu:
        fout.close()
        fout_gt.close()
    return total_correct, total_seen

if __name__ == '__main__':
    with tf.Graph().as_default():
        evaluate()
    LOG_FOUT.close()
