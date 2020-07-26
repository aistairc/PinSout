import numpy as np
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

all_pred_data_label_filelist = os.path.join(BASE_DIR, '/home/dprt/pointnet/sem_seg/log5/output_filelist.txt')
# output_filelist : log/area_1_visu/Area_1_conferenceRoom_1_pred.txt
pred_data_label_filenames = [line.rstrip() for line in open(all_pred_data_label_filelist)]
gt_label_filenames = [f.rstrip('_pred\.txt') + '_gt.txt' for f in pred_data_label_filenames]
num_room = len(gt_label_filenames)

# range(13) -> range(5)
gt_classes = [0 for _ in range(5)]
positive_classes = [0 for _ in range(5)]
true_positive_classes = [0 for _ in range(5)]
for i in range(num_room):
    print(i)
    data_label = np.loadtxt(pred_data_label_filenames[i])
    pred_label = data_label[:, -1]
    gt_label = np.loadtxt(gt_label_filenames[i])
    print(gt_label.shape)
    for j in range(gt_label.shape[0]):
        gt_l = int(gt_label[j])
        pred_l = int(pred_label[j])
        gt_classes[gt_l] += 1
        positive_classes[pred_l] += 1
        true_positive_classes[gt_l] += int(gt_l == pred_l)


print(gt_classes)
print(positive_classes)
print(true_positive_classes)


print('Overall accuracy: {0}'.format(sum(true_positive_classes)/float(sum(positive_classes))))

print('IoU:')
iou_list = []
for i in range(5):
    iou = true_positive_classes[i]/float(gt_classes[i]+positive_classes[i]-true_positive_classes[i]) 
    print(iou)
    iou_list.append(iou)

# print(sum(iou_list)/13.0)
print(sum(iou_list)/5.0)
