#!/usr/bin/env python3
# coding: utf-8

import rospy
import rosbag
import argparse
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg    import Header
import sensor_msgs.point_cloud2 as pc2
from dataclasses import dataclass, field
from lvx2_parser import LVX2_PARSER


# converter
class LVX2_to_ROSBAG(object):

    # constructor
    def __init__( self, in_file:str, out_file:str, pc2_topic:str='livox_points', pc2_frame_id:str='livox_frame' ):
        # initialize parameters
        self._in_file      = in_file
        self._out_file     = out_file
        self._pc2_topic    = pc2_topic
        self._pc2_frame_id = pc2_frame_id
        
        self._lvx2         = LVX2_PARSER( in_file=self._in_file )
        
        # create ROSBAG file
        _bag = rosbag.Bag(self._out_file, 'w')

        # parameter fields        
        _fields = [
            PointField( 'x',             0, PointField.FLOAT32, 1 ),
            PointField( 'y',             4, PointField.FLOAT32, 1 ),
            PointField( 'z',             8, PointField.FLOAT32, 1 ),
            PointField( 'reflectivity', 12, PointField.FLOAT32, 1 ),
            PointField( 'tag',          16, PointField.UINT32,  1 )
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

            _timestamp       = rospy.Time.from_sec(_frm.timestamp)
            _header          = Header()
            _header.stamp    = _timestamp
            _header.frame_id = self._pc2_frame_id
            _pc2             = pc2.create_cloud(_header, _fields, _points_with_fields)
            _bag.write(self._pc2_topic, _pc2, _timestamp)

        # close rosbag file        
        _bag.close()


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
