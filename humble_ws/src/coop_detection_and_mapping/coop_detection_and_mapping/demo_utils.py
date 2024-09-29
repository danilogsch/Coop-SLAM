import argparse
import sys
import os
import warnings
import zipfile

warnings.filterwarnings("ignore", category=UserWarning)

from easydict import EasyDict as edict
import numpy as np
import wget
import torch
import cv2
import torch.nn.functional as F

from .misc import make_folder, time_synchronized
from .evaluation_utils import decode, post_processing
from .torch_utils import _sigmoid


def parse_demo_configs():
    config_dict = {}
    config_dict['saved_fn'] = 'fpn_resnet_50_semseg'
    config_dict['arch'] = 'fpn_resnet_50'
    config_dict['pretrained_path'] = './checkpoints/fpn_resnet_50_semseg/Model_fpn_resnet_50_semseg_epoch_262.pth'
    config_dict['K'] = 50
    config_dict['gpu_idx'] = 0
    config_dict['peak_thresh'] = 0.3
    config_dict['output_width'] = 608
    config_dict['no_cuda'] = False

    configs = edict(config_dict)
    configs.pin_memory = True
    configs.distributed = False  # For testing on 1 GPU only

    configs.input_size = (608, 608)
    configs.hm_size = (152, 152)
    configs.down_ratio = 4
    configs.max_objects = 50

    configs.imagenet_pretrained = False
    configs.head_conv = 64
    configs.num_classes = 14
    configs.num_center_offset = 2
    configs.num_z = 1
    configs.num_dim = 3
    configs.num_direction = 2  # sin, cos

    configs.heads = {
        'hm_cen': configs.num_classes,
        'cen_offset': configs.num_center_offset,
        'direction': configs.num_direction,
        'z_coor': configs.num_z,
        'dim': configs.num_dim
    }

    ####################################################################
    ##############Dataset, Checkpoints, and results dir configs#########
    ####################################################################
    configs.root_dir = './'
    configs.dataset_dir = os.path.join(configs.root_dir, 'synth_depot_dataset')
    configs.results_dir = os.path.join(configs.root_dir, 'results', configs.saved_fn)
    make_folder(configs.results_dir)

    return configs


def do_detect(configs, model, bevmap): #, is_front):
    # if not is_front:
    #     bevmap = torch.flip(bevmap, [1, 2])

    input_bev_maps = bevmap.unsqueeze(0).to(configs.device, non_blocking=True).float()
    t1 = time_synchronized()
    outputs = model(input_bev_maps)
    outputs['hm_cen'] = _sigmoid(outputs['hm_cen'])
    outputs['cen_offset'] = _sigmoid(outputs['cen_offset'])
    outputs['sem_seg'] = F.softmax(outputs['sem_seg'], dim=1)
    seg_out = outputs['sem_seg'].squeeze(0).cpu().numpy()
    seg_out = np.argmax(seg_out, axis = 0)
    
    # detections size (batch_size, K, 10)
    detections = decode(outputs['hm_cen'], outputs['cen_offset'], outputs['direction'], outputs['z_coor'],
                        outputs['dim'], K=configs.K)
    detections = detections.cpu().numpy().astype(np.float32)
    detections = post_processing(detections, configs.num_classes, configs.down_ratio, configs.peak_thresh)
    t2 = time_synchronized()
    # Inference speed
    fps = 1 / (t2 - t1)

    return detections[0], bevmap, fps, seg_out


def write_credit(img, org_author=(500, 400), text_author='github.com/danilogsch', org_fps=(50, 1000), fps=None):
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8
    color = (255, 255, 255)
    thickness = 2

    cv2.putText(img, text_author, org_author, font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.putText(img, 'Frame Rate: {:.1f} FPS'.format(fps), org_fps, font, fontScale, color, thickness, cv2.LINE_AA)

def write_confidence(img, org_confidence=(500, 400), confidence=None):
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.7
    color = (0, 0, 0)
    thickness = 2
    txt_string = 'Confidences: '+', '.join(confidence)
    #print(type(txt_string))
    cv2.putText(img, txt_string, org_confidence, font, fontScale, color, thickness, cv2.LINE_AA)