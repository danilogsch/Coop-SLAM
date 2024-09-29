from __future__ import annotations
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rosbags.typesys.types import sensor_msgs__msg__PointCloud2, sensor_msgs__msg__PointField
import numpy as np



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
        if connection.topic == 'pandar_xt_32_0_lidar':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            #ros_msg = to_native(msg)
            print(msg)
            # Verify the dtype of the array
            assert msg.data.dtype == np.uint8
            # Verify that each value falls within the range [0, 255]
            assert np.all((msg.data >= 0) & (msg.data <= 255))
            field_ros_msg = to_native(msg.fields[0])
            fields_list.append(field_ros_msg)
            field_ros_msg = to_native(msg.fields[1])
            fields_list.append(field_ros_msg)
            field_ros_msg = to_native(msg.fields[2])
            fields_list.append(field_ros_msg)
            
            #print(fields_list)
            
            

            
            ros_msg.header.frame_id = msg.header.frame_id
            ros_msg.header.stamp.sec = msg.header.stamp.sec
            ros_msg.header.stamp.nanosec = msg.header.stamp.nanosec
            ros_msg.fields = fields_list
            fields_list = []
            ros_msg.height = msg.height
            ros_msg.width = msg.width
            ros_msg.is_bigendian = msg.is_bigendian
            ros_msg.point_step = msg.point_step
            ros_msg.row_step = msg.row_step
            
            ros_msg.data = msg.data.tolist()
            
            ros_msg.is_dense = msg.is_dense
            
            #print(reshaped_array_with_fourth_column)
            
            #print(ros_msg)
            print(msg.data.size)
            print(len(ros_msg.data))
            pc_data = point_cloud2.read_points(ros_msg, field_names=("x", "y", "z"), skip_nans=True)
            # Convert point cloud data to a NumPy array
            pc_array = np.array(list(pc_data), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
            pc_flat = pc_array.flatten()
            print(type(pc_data))
            print(pc_data.ndim)
            print(type(pc_array))
            print(pc_array.ndim)
            print(type(pc_flat))
            print(pc_flat.ndim)
            #print(pc_flat)
            pc_flat.tofile('r2b_hallway_lidar/'+f"{seq:06d}"+'.bin')
            
            #data = np.fromfile('r2b_hallway_lidar/'+f"{seq:06d}"+'.bin', dtype=np.float32).reshape(-1, 3)
            
            #fourth_column_value = 0
            #fourth_column = np.full((data.shape[0], 1), fourth_column_value)
            # Concatenate the fourth column to the right of the reshaped array
            #reshaped_array_with_fourth_column = np.hstack((data, fourth_column))
            # Flatten the array back to 1 dimension
            #flattened_array = reshaped_array_with_fourth_column.flatten()
            #flattened_array.tofile('r2b_hallway_lidar/'+f"{seq:06d}"+'.bin')
            #print(flattened_array.reshape(-1, 4))
            seq +=1
            #break
            
            

    # messages() accepts connection filters
#
