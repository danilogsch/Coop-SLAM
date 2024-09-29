import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os

keys = ['0_sp', '0_sr', '0_iou', '0_xerr', '0_yerr', '0_yawerr', 
        '1_sp', '1_sr', '1_iou', '1_xerr', '1_yerr', '1_yawerr', 
        '2_sp', '2_sr', '2_iou', '2_xerr', '2_yerr', '2_yawerr', 
        '3_sp', '3_sr', '3_iou', '3_xerr', '3_yerr', '3_yawerr', 
        '4_sp', '4_sr', '4_iou', '4_xerr', '4_yerr', '4_yawerr', 
        '5_sp', '5_sr', '5_iou', '5_xerr', '5_yerr', '5_yawerr', 
        '6_sp', '6_sr', '6_iou', '6_xerr', '6_yerr', '6_yawerr', 
        '7_sp', '7_sr', '7_iou', '7_xerr', '7_yerr', '7_yawerr', 
        '8_sp', '8_sr', '8_iou', '8_xerr', '8_yerr', '8_yawerr', 
        '9_sp', '9_sr', '9_iou', '9_xerr', '9_yerr', '9_yawerr', 
        '10_sp', '10_sr', '10_iou', '10_xerr', '10_yerr', '10_yawerr', 
        '11_sp', '11_sr', '11_iou', '11_xerr', '11_yerr', '11_yawerr', 
        '12_sp', '12_sr', '12_iou', '12_xerr', '12_yerr', '12_yawerr', 
        '13_sp', '13_sr', '13_iou', '13_xerr', '13_yerr', '13_yawerr', 
        '14_sp', '14_sr', '14_iou', '14_xerr', '14_yerr', '14_yawerr', 
        'total_sp', 'total_sr', 'total_iou']

CLASS_ID_TO_COLOR = {
    '0': [212,12,128],
    '1': [190,18,192],
    '2': [168,24,0],
    '3': [146,30,64],
    '4': [124,36,128],
    '5': [102,42,192],
    '6': [80,48,0],
    '7': [58,54,64],
    '8': [36,60,128],
    '9': [14,66,192],
    '10': [248,72,0],
    '11': [226,78,64],
    '12': [204,84,128],
    '13': [182,90,192]
}

def calc_iou_ss(pred_mask,gt_mask, cls):
    tp = 0
    fp = 0
    fn = 0
    iou = 0
    # pixels_pred, _ = np.where(pred_mask==1)
    preds = len(np.where(pred_mask==1)[0])
    # pixels_tp, _ = np.where((np.bitwise_and(pred_mask, gt_mask))==1)
    union = len(np.where((np.bitwise_or(pred_mask, gt_mask))==1)[0])
    tp = len(np.where((np.bitwise_and(pred_mask, gt_mask))==1)[0])
    fp = preds - tp
    # gt_lbs, _ = np.where(gt_mask==1)
    fn = len(np.where(gt_mask==1)[0]) - tp
    if (tp+fp) > 0:
        sap = tp/(tp+fp)
    else:
        sap = 0
    if (tp+fn) > 0:
        sar = tp/(tp+fn)
    else:
        sar = 0
    if union > 0:
        iou = tp/union
    else:
        iou = 0
    return iou, sap, sar

# Create a dictionary of empty arrays
evaluation_dict = {key: [] for key in keys}


gt_idx = 4
pred_idx = 1

t = 0.5
time = []
x_err = []
y_err = []
yaw_err = []

# Get the value of an environment variable
coop_slam_path = os.getenv('COOP_SLAM_PATH')

if coop_slam_path:
    print(f"The path is: {coop_slam_path}")
else:
    print("Environment variable COOP_SLAM_PATH not found.")

for i in range(96):
    print(i)
    sample_error_x = []
    sample_error_y = []
    sample_error_yaw = []
    gt_txt = coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/groundtruths/poses/{gt_idx:06d}.txt'
    pred_txt = coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/ogms/poses/{pred_idx:06d}.txt'
    gt_seg = coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/groundtruths/ogm/{gt_idx:06d}.png'
    pred_seg = coop_slam_path+f'/humble_ws/src/coop_detection_and_mapping/ogms/ogm/{pred_idx:06d}.png'

    sem_label = cv2.cvtColor(cv2.imread(gt_seg), cv2.COLOR_BGR2RGB)
    seg_out = cv2.cvtColor(cv2.imread(pred_seg), cv2.COLOR_BGR2RGB)

    sp_array = []
    sr_array = []
    iou_array= []

    for j in range(14): #SEM. SEG. Precision, Recall, IoU calc (152x152)
        mask_pred = cv2.cvtColor(np.zeros_like(seg_out, dtype=np.uint8),cv2.COLOR_RGB2GRAY)
        mask_pred[np.where((seg_out==CLASS_ID_TO_COLOR[f'{j}']).all(axis = 2))] = 1
        mask_label = cv2.cvtColor(np.zeros_like(sem_label, dtype=np.uint8),cv2.COLOR_RGB2GRAY)
        mask_label[np.where((sem_label==CLASS_ID_TO_COLOR[f'{j}']).all(axis = 2))] = 1
        if len(np.where((seg_out==CLASS_ID_TO_COLOR[f'{j}']).all(axis = 2))[0]) == 0 and len(np.where((sem_label==CLASS_ID_TO_COLOR[f'{j}']).all(axis = 2))[0]) == 0:
            print('skip iteration')
            continue

        # # for u, v in np.transpose(np.where(seg_out==j)):
        # #     mask_pred[u,v] = 1
        iou, sap, sar = calc_iou_ss(mask_pred, mask_label, j)

        #STORE PER CLASS/PER SAMPLE
        if iou > 0:
            evaluation_dict[f'{j}_iou'].append(iou)
            iou_array.append(iou)
        if sap > 0:
            evaluation_dict[f'{j}_sp'].append(sap)
            sp_array.append(sap)
        if sar > 0:
            evaluation_dict[f'{j}_sr'].append(sar)
            sr_array.append(sar)

    evaluation_dict[f'total_iou'].append(np.mean(iou_array))
    evaluation_dict[f'total_sp'].append(np.mean(sp_array))
    evaluation_dict[f'total_sr'].append(np.mean(sr_array))

    sym_cls = [1,5,6,13] #symmetric classes array

    # Open the file in read mode
    with open(gt_txt, "r") as file:
    # Read all lines into a list
        lines = file.readlines()

    with open(pred_txt, "r") as file:
    # Read all lines into a list
        lines_pred = file.readlines()

    for line in lines[1:]:
        #print(line)
        line = line.rstrip()
        line_parts = line.split(' ')
        cls_gt = int(float(line_parts[1]))
        if cls_gt == 9:
            continue
        x_gt = float(line_parts[2])
        y_gt = float(line_parts[3])
        yaw_gt = float(line_parts[4])
        if yaw_gt<-math.pi or yaw_gt>math.pi:
            print('yaw gt out of bound')
        flag = False
        for line_pred in lines_pred:
            line_pred = line_pred.rstrip()
            pred_parts = line_pred.split(' ')
            cls_pred = int(float(pred_parts[1]))
            if cls_pred == 9 or cls_pred != cls_gt:
                continue
            x_pred = float(pred_parts[2])
            y_pred = float(pred_parts[3])
            yaw_pred = float(pred_parts[4])
            if yaw_pred<-math.pi or yaw_pred>math.pi:
                print('yaw pred out of bound')

            distance = math.sqrt((x_pred - x_gt)**2 + (y_pred - y_gt)**2)
            if distance < 0.5:
                print('match found')
                flag = True

                evaluation_dict[f'{cls_pred}_xerr'].append(abs(x_gt - x_pred))
                evaluation_dict[f'{cls_pred}_yerr'].append(abs(y_gt - y_pred))
                evaluation_dict[f'{cls_pred}_yawerr'].append(abs(np.mod((yaw_gt - yaw_pred)+np.pi,2*np.pi)-np.pi)) #mudar aqui -pi pi

                if abs(np.mod((yaw_gt - yaw_pred)+np.pi,2*np.pi)-np.pi) > 1.5708 and cls_pred in sym_cls:
                    yaw_error_update = abs(np.mod((yaw_gt - (np.mod((yaw_pred)+2*np.pi,2*np.pi)-np.pi))+np.pi,2*np.pi)-np.pi)
                    sample_error_yaw.append(yaw_error_update) #mudar aqui -pi pi
                else:
                    sample_error_yaw.append(abs(np.mod((yaw_gt - yaw_pred)+np.pi,2*np.pi)-np.pi))
                sample_error_x.append(abs(x_gt - x_pred))
                sample_error_y.append(abs(y_gt - y_pred))
                #sample_error_yaw.append(abs(np.mod((yaw_gt - yaw_pred)+np.pi,2*np.pi)-np.pi)) #mudar aqui -pi pi

    if flag and (cls_pred != 9 or cls_pred !=10):
        print('NO MATCH FOUND! False negative') #Not supposed to happen

    #AVG sample error
    x_err.append(np.mean(sample_error_x))
    y_err.append(np.mean(sample_error_y))
    yaw_err.append(np.mean(sample_error_yaw))


    time.append(t)
    t += 0.1
    gt_idx += 1
    pred_idx += 1

# Plot X vs. Time
plt.subplot(3, 1, 1)
plt.plot(time, x_err, marker='o')
plt.title('X vs. Time')
plt.xlabel('Time')
plt.ylabel('X')

# Plot Y vs. Time
plt.subplot(3, 1, 2)
plt.plot(time, y_err, marker='o')
plt.title('Y vs. Time')
plt.xlabel('Time')
plt.ylabel('Y')

# Plot Yaw vs. Time
plt.subplot(3, 1, 3)
plt.plot(time, yaw_err, marker='o')
plt.title('Yaw vs. Time')
plt.xlabel('Time')
plt.ylabel('Yaw')

plt.tight_layout()  # Adjust layout to prevent overlapping
plt.show()

for key in evaluation_dict:
    if len(evaluation_dict[key])>0:
        evaluation_dict[key] = np.mean(evaluation_dict[key])*100
    else:
        evaluation_dict[key] = 0
print(evaluation_dict)

mean_x_err = np.mean(x_err)
mean_y_err = np.mean(y_err)
mean_yaw_err = np.mean(yaw_err)

print(f'Mean X, Y, YAW errors: {mean_x_err} {mean_y_err} {mean_yaw_err}')

print('Finished Evaluation!')