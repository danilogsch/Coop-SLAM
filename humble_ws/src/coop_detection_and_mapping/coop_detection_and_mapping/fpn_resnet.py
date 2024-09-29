from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os

import torch
import torch.nn as nn
import torch.utils.model_zoo as model_zoo
import torch.nn.functional as F

BN_MOMENTUM = 0.1

model_urls = {
    'resnet18': 'https://download.pytorch.org/models/resnet18-5c106cde.pth',
    'resnet34': 'https://download.pytorch.org/models/resnet34-333f7ec4.pth',
    'resnet50': 'https://download.pytorch.org/models/resnet50-19c8e357.pth',
    'resnet101': 'https://download.pytorch.org/models/resnet101-5d3b4d8f.pth',
    'resnet152': 'https://download.pytorch.org/models/resnet152-b121ed2d.pth',
}


def conv3x3(in_planes, out_planes, stride=1):
    """3x3 convolution with padding"""
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride, padding=1, bias=False)


class BasicBlock(nn.Module):
    expansion = 1

    def __init__(self, inplanes, planes, stride=1, downsample=None):
        super(BasicBlock, self).__init__()
        self.conv1 = conv3x3(inplanes, planes, stride)
        self.bn1 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = conv3x3(planes, planes)
        self.bn2 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        residual = x

        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)

        return out


class Bottleneck(nn.Module):
    expansion = 4

    def __init__(self, inplanes, planes, stride=1, downsample=None):
        super(Bottleneck, self).__init__()
        self.conv1 = nn.Conv2d(inplanes, planes, kernel_size=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
        self.conv3 = nn.Conv2d(planes, planes * self.expansion, kernel_size=1, bias=False)
        self.bn3 = nn.BatchNorm2d(planes * self.expansion, momentum=BN_MOMENTUM)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        residual = x

        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)

        out = self.conv3(out)
        out = self.bn3(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)

        return out


class PoseResNet(nn.Module):

    def __init__(self, block, layers, heads, head_conv, **kwargs):
        self.inplanes = 64
        self.deconv_with_bias = False
        self.heads = heads

        super(PoseResNet, self).__init__()
        self.conv1 = nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = nn.BatchNorm2d(64, momentum=BN_MOMENTUM)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        self.layer1 = self._make_layer(block, 64, layers[0])
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2)
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2)
        self.layer4 = self._make_layer(block, 512, layers[3], stride=2)

        self.conv_up_level1 = nn.Conv2d(768* block.expansion, 256* block.expansion, kernel_size=1, stride=1, padding=0)
        self.conv_up_level2 = nn.Conv2d(384* block.expansion, 128* block.expansion, kernel_size=1, stride=1, padding=0)
        self.conv_up_level3 = nn.Conv2d(192* block.expansion, 64* block.expansion, kernel_size=1, stride=1, padding=0)

        fpn_channels = [256* block.expansion, 128* block.expansion, 64* block.expansion]
        for fpn_idx, fpn_c in enumerate(fpn_channels):
            for head in sorted(self.heads):
                num_output = self.heads[head]
                if head_conv > 0:
                    fc = nn.Sequential(
                        nn.Conv2d(fpn_c, head_conv, kernel_size=3, padding=1, bias=True),
                        nn.ReLU(inplace=True),
                        nn.Conv2d(head_conv, num_output, kernel_size=1, stride=1, padding=0))
                else:
                    fc = nn.Conv2d(in_channels=fpn_c, out_channels=num_output, kernel_size=1, stride=1, padding=0)

                self.__setattr__('fpn{}_{}'.format(fpn_idx, head), fc)

        self.fseg_top = nn.Sequential(
                        nn.Conv2d(2048, 512, 3, padding=1, bias=False),
                        nn.BatchNorm2d(512),
                        nn.ReLU(),
                        nn.Dropout(0.1),
                        nn.Conv2d(512, 15, 1),
                        # First transposed convolution layer (upscale by 2)
                        # nn.ConvTranspose2d(in_channels=64, out_channels=128, kernel_size=3, stride=2, padding=1),
                        # nn.BatchNorm2d(128, momentum=BN_MOMENTUM),
                        # nn.ReLU(inplace=True),
        )
        self.fseg_mid = nn.Sequential(
                        nn.Conv2d(1024, 256, 3, padding=1, bias=False),
                        nn.BatchNorm2d(256),
                        nn.ReLU(),
                        nn.Dropout(0.1),
                        nn.Conv2d(256, 15, 1),
        )
        self.fseg_bot = nn.Sequential(
                        nn.Conv2d(512, 128, 3, padding=1, bias=False),
                        nn.BatchNorm2d(128),
                        nn.ReLU(),
                        nn.Dropout(0.1),
                        nn.Conv2d(128, 15, 1),
        )
        #self.conv_up_seg = nn.Conv2d(192, 128, kernel_size=1, stride=1, padding=0)
        self.deconv_up_seg_top = nn.ConvTranspose2d(in_channels=15, out_channels=15, kernel_size=3, stride=2, padding=1,dilation=1)
        self.bn_seg_top = nn.BatchNorm2d(15)
        self.deconv_up_seg_mid = nn.ConvTranspose2d(in_channels=15, out_channels=15, kernel_size=3, stride=2, padding=1,dilation=1)
        self.bn_seg_mid = nn.BatchNorm2d(15)
        self.deconv_up_seg_bot = nn.ConvTranspose2d(in_channels=15, out_channels=15, kernel_size=3, stride=2, padding=1,dilation=1)
#                         # Second transposed convolution layer (upscale by 2)
#                         nn.ConvTranspose2d(in_channels=64, out_channels=15, kernel_size=3, stride=2, padding=1),
                        
# )

    def _make_layer(self, block, planes, blocks, stride=1):
        downsample = None
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                nn.Conv2d(self.inplanes, planes * block.expansion, kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(planes * block.expansion, momentum=BN_MOMENTUM),
            )

        layers = []
        layers.append(block(self.inplanes, planes, stride, downsample))
        self.inplanes = planes * block.expansion
        for i in range(1, blocks):
            layers.append(block(self.inplanes, planes))

        return nn.Sequential(*layers)

    def forward(self, x):
        _, _, input_h, input_w = x.size()
        hm_h, hm_w = input_h // 4, input_w // 4
        x = self.conv1(x)
        x = self.bn1(x)
        x_s = self.relu(x)
        x = self.maxpool(x_s)

        out_layer1 = self.layer1(x)
        out_layer2 = self.layer2(out_layer1)

        out_layer3 = self.layer3(out_layer2)

        out_layer4 = self.layer4(out_layer3)

        # up_level1: torch.Size([b, 2048, 38, 38])
        up_level1 = F.interpolate(out_layer4, scale_factor=2, mode='bilinear', align_corners=True)
        # concat_level1: torch.Size([b, 3078, 38, 38])
        concat_level1 = torch.cat((up_level1, out_layer3), dim=1)
        # up_level2: torch.Size([b, 1024, 76, 76])
        up_level2 = F.interpolate(self.conv_up_level1(concat_level1), scale_factor=2, mode='bilinear',
                                  align_corners=True)
        # concat_level2: torch.Size([b, 1536, 76, 76])
        concat_level2 = torch.cat((up_level2, out_layer2), dim=1)
        # up_level3: torch.Size([b, 512, 152, 152]),
        up_level3 = F.interpolate(self.conv_up_level2(concat_level2), scale_factor=2, mode='bilinear',
                                  align_corners=True)
        # up_level4: torch.Size([b, 256, 152, 152])
        up_level4 = self.conv_up_level3(torch.cat((up_level3, out_layer1), dim=1))

        ret = {}
        for head in self.heads:
            temp_outs = []
            for fpn_idx, fdn_input in enumerate([up_level2, up_level3, up_level4]):
                fpn_out = self.__getattr__('fpn{}_{}'.format(fpn_idx, head))(fdn_input)
                _, _, fpn_out_h, fpn_out_w = fpn_out.size()
                # Make sure the added features having same size of heatmap output
                if (fpn_out_w != hm_w) or (fpn_out_h != hm_h):
                    fpn_out = F.interpolate(fpn_out, size=(hm_h, hm_w))
                temp_outs.append(fpn_out)
            # Take the softmax in the keypoint feature pyramid network
            final_out = self.apply_kfpn(temp_outs)

            ret[head] = final_out

        predict1 = self.relu(self.fseg_top(out_layer4))
        predict2 = self.relu(self.fseg_mid(out_layer3))
        predict3 = self.relu(self.fseg_bot(out_layer2))
        predict2_up = self.bn_seg_top(F.interpolate(self.deconv_up_seg_top(predict1), size=(38,38))+F.interpolate(predict2, size=(38,38)))
        predict3_up = self.bn_seg_mid(F.interpolate(self.deconv_up_seg_mid(predict2_up),size=(76,76))+F.interpolate(predict3,size=(76,76)))
        seg_out = F.interpolate(self.deconv_up_seg_bot(predict3_up),size=(152,152))
        #ret['seg_head']
        # seg_level_1 = self.fseg(up_level4)
        # seg_level_1 = F.interpolate(seg_level_1, size=(304, 304))
        # seg_level_2 = self.conv_up_seg(torch.cat((seg_level_1, x_s), dim=1))
        # seg_out = self.deconv_up_seg(seg_level_2)


        ret['sem_seg'] = seg_out
        
        return ret

    def apply_kfpn(self, outs):
        outs = torch.cat([out.unsqueeze(-1) for out in outs], dim=-1)
        softmax_outs = F.softmax(outs, dim=-1)
        ret_outs = (outs * softmax_outs).sum(dim=-1)
        return ret_outs

    def init_weights(self, num_layers, pretrained=True):
        if pretrained:
            # TODO: Check initial weights for head later
            for fpn_idx in [0, 1, 2]:  # 3 FPN layers
                for head in self.heads:
                    final_layer = self.__getattr__('fpn{}_{}'.format(fpn_idx, head))
                    for i, m in enumerate(final_layer.modules()):
                        if isinstance(m, nn.Conv2d):
                            # nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                            # print('=> init {}.weight as normal(0, 0.001)'.format(name))
                            # print('=> init {}.bias as 0'.format(name))
                            if m.weight.shape[0] == self.heads[head]:
                                if 'hm' in head:
                                    nn.init.constant_(m.bias, -2.19)
                                else:
                                    nn.init.normal_(m.weight, std=0.001)
                                    nn.init.constant_(m.bias, 0)
            #ADD SEM SEG WEIGHTS
            for i, m in enumerate(self.fseg_top.modules()):
                if isinstance(m, nn.Conv2d):
                    nn.init.normal_(m.weight, std=0.001)
            for i, m in enumerate(self.fseg_mid.modules()):
                if isinstance(m, nn.Conv2d):
                    nn.init.normal_(m.weight, std=0.001)
            for i, m in enumerate(self.fseg_bot.modules()):
                if isinstance(m, nn.Conv2d):
                    nn.init.normal_(m.weight, std=0.001)
            for i, m in enumerate(self.deconv_up_seg_top.modules()):
                if isinstance(m, nn.ConvTranspose2d):
                    if m.weight.shape[0] == 15:
                        nn.init.normal_(m.weight, std=0.001)
                        nn.init.constant_(m.bias, 0)
                    else:
                        nn.init.normal_(m.weight, std=0.001)
                        nn.init.constant_(m.bias, 0)
            for i, m in enumerate(self.deconv_up_seg_mid.modules()):
                if isinstance(m, nn.ConvTranspose2d):
                    if m.weight.shape[0] == 15:
                        nn.init.normal_(m.weight, std=0.001)
                        nn.init.constant_(m.bias, 0)
                    else:
                        nn.init.normal_(m.weight, std=0.001)
                        nn.init.constant_(m.bias, 0)
            for i, m in enumerate(self.deconv_up_seg_bot.modules()):
                if isinstance(m, nn.ConvTranspose2d):
                    if m.weight.shape[0] == 15:
                        nn.init.normal_(m.weight, std=0.001)
                        nn.init.constant_(m.bias, 0)
                    else:
                        nn.init.normal_(m.weight, std=0.001)
                        nn.init.constant_(m.bias, 0)

            # pretrained_state_dict = torch.load(pretrained)
            url = model_urls['resnet{}'.format(num_layers)]
            pretrained_state_dict = model_zoo.load_url(url)
            print('=> loading pretrained model {}'.format(url))
            self.load_state_dict(pretrained_state_dict, strict=False)


resnet_spec = {18: (BasicBlock, [2, 2, 2, 2]),
               34: (BasicBlock, [3, 4, 6, 3]),
               50: (Bottleneck, [3, 4, 6, 3]),
               101: (Bottleneck, [3, 4, 23, 3]),
               152: (Bottleneck, [3, 8, 36, 3])}


def get_pose_net(num_layers, heads, head_conv, imagenet_pretrained):
    block_class, layers = resnet_spec[num_layers]

    model = PoseResNet(block_class, layers, heads, head_conv=head_conv)
    model.init_weights(num_layers, pretrained=imagenet_pretrained)
    return model