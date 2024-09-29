from __future__ import annotations
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rosbags.typesys.types import sensor_msgs__msg__PointCloud2, sensor_msgs__msg__PointField
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



import importlib
from typing import TYPE_CHECKING

import numpy

if TYPE_CHECKING:
    from typing import Any

NATIVE_CLASSES: dict[str, Any] = {}


def to_native(msg: Any) -> Any:  # noqa: ANN401
    """Convert rosbags message to native message.

    Args:
        msg: Rosbags message.

    Returns:
        Native message.

    """
    msgtype: str = msg.__msgtype__
    if msgtype not in NATIVE_CLASSES:
        pkg, name = msgtype.rsplit('/', 1)
        NATIVE_CLASSES[msgtype] = getattr(importlib.import_module(pkg.replace('/', '.')), name)

    fields = {}
    for name, field in msg.__dataclass_fields__.items():
        if 'ClassVar' in field.type:
            continue
        value = getattr(msg, name)
        if '__msg__' in field.type:
            value = to_native(value)
        elif isinstance(value, list):
            value = [to_native(x) for x in value]
        elif isinstance(value, numpy.ndarray):
            value = value.tolist()
        fields[name] = value

    return NATIVE_CLASSES[msgtype](**fields)

br = CvBridge()
# create reader instance and open for reading
with Reader('r2b_hallway') as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)
        
    ros_msg = PointCloud2()
    seq = 0
    fields_list = []

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == 'hawk_0_left_rgb_image':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            #ros_msg = to_native(msg)
            print(msg)
            ros_msg = to_native(msg)
            cv_msg = br.imgmsg_to_cv2(ros_msg)
            #print(cv_msg)
            cv2.imwrite(f'/home/danilo/r2b_dataset/r2b_hallway_rgb/{seq:06d}.png', cv_msg)
            seq += 1
            
            
            

    # messages() accepts connection filters
#
