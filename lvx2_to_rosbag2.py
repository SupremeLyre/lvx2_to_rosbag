#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
import struct
import argparse
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg    import Header
from sensor_msgs_py import point_cloud2
from dataclasses import dataclass, field
from builtin_interfaces.msg import Time

from lvx2_parser import LVX2_PARSER



# converter
class LVX2_to_ROSBAG2(object):
    # constructor
    def __init__( self, in_file:str, out_file:str, pc2_topic:str='livox_points', pc2_frame_id:str='livox_frame' ):
        # initialize parameters
        self._in_file      = in_file
        self._out_file     = out_file
        self._pc2_topic    = pc2_topic
        self._pc2_frame_id = pc2_frame_id

        # headers
        self._pub_header = PublicHeader()
        self._prv_header = PrivateHeader()
        self._devices    = []
        self._frames     = []
        
        self._lvx2       = LVX2_PARSER( in_file=self._in_file )

        # create ROSBAG file
        storage_options = rosbag2_py.StorageOptions(
            uri        = self._out_file,
            storage_id = 'sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format  = 'cdr',
            output_serialization_format = 'cdr'
        )
        
        _bag = rosbag2_py.SequentialWriter()
        _bag.open(storage_options, converter_options)
        
        # add PointCloud2 Topic
        _topic_info = rosbag2_py.TopicMetadata(
            name                 = self._pc2_topic,
            type                 = 'sensor_msgs/msg/PointCloud2',
            serialization_format = 'cdr'
        )
        _bag.create_topic(_topic_info)
        
        # parameter fields
        _fields = [
            PointField( name='x',            offset= 0, datatype=PointField.FLOAT32, count=1 ),
            PointField( name='y',            offset= 4, datatype=PointField.FLOAT32, count=1 ),
            PointField( name='z',            offset= 8, datatype=PointField.FLOAT32, count=1 ),
            PointField( name='reflectivity', offset=12, datatype=PointField.FLOAT32, count=1 ),
            PointField( name='tag',          offset=16, datatype=PointField.UINT32,  count=1 )
        ]

        i = 0
        _frm_size = len(self._lvx2._frames)
        for _frm in self._lvx2._frames:
            i = i + 1
            print('processing frames ({0}/{1})'.format(i, _frm_size))

            _ts = 0.0
            _points_with_fields = []
            for _pkg in _frm.packages:
                for _p in _pkg.points:
                    _points_with_fields.append( [_p.x, _p.y, _p.z, _p.reflectivity, _p.tag] )

            _timestamp         = Time()
            _timestamp.sec     = int(_frm.timestamp)
            _timestamp.nanosec = int((_frm.timestamp - _timestamp.sec) * 1e9)
            _header          = Header()
            _header.stamp    = _timestamp
            _header.frame_id = self._pc2_frame_id
            _pc2             = point_cloud2.create_cloud(_header, _fields, _points_with_fields)
            _bag.write(self._pc2_topic, serialize_message(_pc2), int(_frm.timestamp*1e9))


    # destructor
    def __del__(self):
        pass




# main function
def main(args=None):
    _parser = argparse.ArgumentParser(description="convert lvx2 to bag(for ROS1) script")
    _parser.add_argument('--in_file',      type=str, help='input lvx2 file',    required=True)
    _parser.add_argument('--out_file',     type=str, help='output rosbag file', required=True)
    _parser.add_argument('--pc2_topic',    type=str, help='PointCloud2 topic name', default='livox_points')
    _parser.add_argument('--pc2_frame_id', type=str, help='PointCloud2 frame name', default='livox_frame')
    _args = _parser.parse_args(args)

    _input_file   = _args.in_file
    _output_file  = _args.out_file
    _pc2_topic    = _args.pc2_topic
    _pc2_frame_id = _args.pc2_frame_id

    bag = LVX2_to_ROSBAG( in_file=_input_file, out_file=_output_file, pc2_topic=_pc2_topic, pc2_frame_id=_pc2_frame_id )
    return


# entry point
if( __name__ == '__main__' ):
    main()
