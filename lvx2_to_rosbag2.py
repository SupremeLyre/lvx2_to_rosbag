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


@dataclass
class PublicHeader:
    signature  : str = ''
    ver_a      : int = 0
    ver_b      : int = 0
    ver_c      : int = 0
    ver_d      : int = 0
    magic_code : int = 0


@dataclass
class PrivateHeader:
    duration     : int = 0
    device_count : int = 0


@dataclass
class DeviceInformation:
    lidar_sn         : str = ''
    hub_sn           : str = ''
    lidar_id         : int = 0
    lidar_type       : int = 0
    device_type      : int = 0
    enable_extrinsic : int = 0
    offset_roll      : float = 0.0
    offset_pitch     : float = 0.0
    offset_yaw       : float = 0.0
    offset_x         : float = 0.0
    offset_y         : float = 0.0
    offset_z         : float = 0.0


@dataclass
class FrameHeader:
    current_offset : int = 0
    next_offset    : int = 0
    frame_index    : int = 0
    frame_size     : int = 0


@dataclass
class PackageHeader:
    version        : int   = 0
    lidar_id       : int   = 0
    lidar_type     : int   = 0
    timestamp_type : int   = 0
    timestamp      : float = 0.0
    udp_count      : int   = 0
    data_type      : int   = 0
    length         : int   = 0
    frame_count    : int   = 0
    points_count   : int   = 0


@dataclass
class Point:
    x            : float = 0.0
    y            : float = 0.0
    z            : float = 0.0
    reflectivity : int   = 0
    tag          : int   = 0


@dataclass
class Package:
    header : PackageHeader = PackageHeader()
    points : list          = field(default_factory=list)


@dataclass
class Frame:
    header    : FrameHeader = FrameHeader()
    timestamp : int         = 0
    packages  : list        = field(default_factory=list)




# pop n items from list object (FIFO)
def pop_n( size:int, data:list ):
    return ( data[:size], data[size:] )


# converter
class LVX2_to_ROSBAG(object):
    # parse public header
    def parse_public_header( self, buf : bytes ):
        if( len(buf) != 24 ):
            print('invalid data length')
            return None

        ret = PublicHeader()
        ret.signature  = buf[0:16].decode(encoding='utf-8', errors='ignore')
        ret.ver_a      = int(struct.unpack( '<b', buf[16:17] )[0])
        ret.ver_b      = int(struct.unpack( '<b', buf[17:18] )[0])
        ret.ver_c      = int(struct.unpack( '<b', buf[18:19] )[0])
        ret.ver_d      = int(struct.unpack( '<b', buf[19:20] )[0])
        ret.magic_code = int(struct.unpack( '<L', buf[20:24] )[0])

        # sigunature check
        if( not ret.signature.startswith('livox_tech') ):
            print('Sigunature Error (livox_tech) :', ret.signature)
            return None
        else:
            print('Signature OK :', ret.signature)

        # magic check
        if( ret.magic_code != 0xAC0EA767 ):
            print('Magic Code Error (0xAC0EA767) : ', ret.magic_code)
            return None
        else:
            print('Magic Code OK :', hex(ret.magic_code))

        return ret


    # parse private header
    def parse_private_header( self, buf : bytes ):
        if( len(buf) != 5 ):
            print('invalid data length')
            return None

        ret = PrivateHeader()
        ret.duration     = float(struct.unpack( '<L', buf[0:4] )[0]) / 1.0e3  # [sec] duration time
        ret.device_count = int(  struct.unpack( '<b', buf[4:5] )[0])            # device counts
        print( 'Data duration :', ret.duration   )
        print( 'Devices       :', ret.device_count )

        return ret


    # device information
    def parse_device_information( self, buf : bytes ):
        if( len(buf) != 63 ):
            print('invalid data length')
            return None

        ret = DeviceInformation()
        ret.lidar_sn         = buf[0:16].decode(encoding='utf-8', errors='ignore')  # LiDAR S/N
        ret.hub_sn           = buf[16:32].decode(encoding='utf-8', errors='ignore') # device S/N
        ret.lidar_id         = int(struct.unpack( '<L', buf[32:36] )[0])   # LiDAR ID
        ret.lidar_type       = int(struct.unpack( '<b', buf[36:37] )[0])   # LiDAR type
        ret.device_type      = int(struct.unpack( '<b', buf[37:38] )[0])   # device type
        ret.enable_extrinsic = int(struct.unpack( '<b', buf[38:39] )[0])   # extrinsic enable
        ret.offset_roll      = float(struct.unpack( '<f', buf[39:43] )[0]) # [deg] offset roll(X axis)
        ret.offset_pitch     = float(struct.unpack( '<f', buf[43:47] )[0]) # [deg] offset pitch(Y axis)
        ret.offset_yaw       = float(struct.unpack( '<f', buf[47:51] )[0]) # [deg] offset yaw(Z axis)
        ret.offset_x         = float(struct.unpack( '<f', buf[51:55] )[0]) # [m] offset X
        ret.offset_y         = float(struct.unpack( '<f', buf[55:59] )[0]) # [m] offset Y
        ret.offset_z         = float(struct.unpack( '<f', buf[59:63] )[0]) # [m] offset Z

        print( 'LiDAR  SN   :', ret.lidar_sn )
        print( 'HUB SN      :', ret.hub_sn   )
        print( 'LiDAR ID    :', ret.lidar_id )
        if( ret.device_type == 9 ):
            print( 'Device Type : MID-360' )
        elif( ret.device_type == 10 ):
            print( 'Device Type : HAP' )
        else:
            print( 'Device Type : Unknown' )

        if( ret.enable_extrinsic ):
            print( 'Extrinsic   : True' )
        else:
            print( 'Extrinsic   : False' )
        print( 'offset rotation : ', (ret.offset_roll, ret.offset_pitch, ret.offset_yaw) )
        print( 'offset position : ', (ret.offset_x,    ret.offset_y,     ret.offset_z  ) )

        return ret


    # frame header
    def parse_frame_header( self, buf : bytes ):
        if( len(buf) != 24 ):
            print('invalid data length')
            return None

        ret = FrameHeader()
        ret.current_offset = struct.unpack( '<Q', buf[0:8]   )[0]
        ret.next_offset    = struct.unpack( '<Q', buf[8:16]  )[0]
        ret.frame_index    = struct.unpack( '<Q', buf[16:24] )[0]
        ret.frame_size     = ret.next_offset - ret.current_offset
        print( 'current offset : ', ret.current_offset )
        print( 'next offset    : ', ret.next_offset    )
        print( 'frame index    : ', ret.frame_index    )
        print( 'frame size     : ', ret.frame_size     )

        return ret


    # package header
    def parse_package_header( self, buf : bytes ):
        if( len(buf) != 27 ):
            print('invalid data length')
            return None
        ret = PackageHeader()
        ret.version        = int(struct.unpack(   '<b', buf[0:1]   )[0])
        ret.lidar_id       = int(struct.unpack(   '<L', buf[1:5]   )[0])
        ret.lidar_type     = int(struct.unpack(   '<b', buf[5:6]   )[0])
        ret.timestamp_type = int(struct.unpack(   '<b', buf[6:7]   )[0])
        ret.timestamp      = float(struct.unpack( '<Q', buf[7:15]  )[0]) / 1.0e9
        ret.udp_count      = int(struct.unpack(   '<H', buf[15:17] )[0])
        ret.data_type      = int(struct.unpack(   '<b', buf[17:18] )[0])
        ret.length         = int(struct.unpack(   '<L', buf[18:22] )[0])
        ret.frame_count    = int(struct.unpack(   '<b', buf[22:23] )[0])

        # check data type
        if(   ret.data_type == 1 ):
            data_length = 14
        elif( ret.data_type == 2 ):
            data_length = 8
        else:
            print( 'invalid data type' )
            return None

        # check data size
        if( ret.length % data_length != 0 ):
            print( 'invalid point data size' )
            return None

        ret.points_count   = ret.length / data_length

        """
        print( 'protocol version  :', ret.version )
        print( 'LiDAR ID          :', ret.lidar_id )
        print( 'LiDAR type        :', ret.lidar_type )
        print( 'timestamp type    :', ret.timestamp_type )
        print( 'timestamp         :', ret.timestamp )
        print( 'UDP count         :', ret.udp_count )
        print( 'data type         :', ret.data_type )
        print( 'point data length :', ret.length )
        print( 'frame count       :', ret.frame_count )
        print( 'points count      :', ret.points_count )
        """

        return ret


    # points
    def parse_points( self, buf : bytes, data_type : int ):
        if(   data_type == 1 ):
            data_length = 14
        elif( data_type == 2 ):
            data_length = 8
        else:
            print( 'invalid data type' )
            return None

        ret = []
        for _ in range( int(len(buf)/data_length) ):
            p = Point()
            (_pbuf, buf) = pop_n(data_length, buf)
            if( data_type == 1 ):
                p.x            = struct.unpack( '<l', _pbuf[0:4]   )[0] / 1.0e3
                p.y            = struct.unpack( '<l', _pbuf[4:8]   )[0] / 1.0e3
                p.z            = struct.unpack( '<l', _pbuf[8:12]  )[0] / 1.0e3
                p.reflectivity = struct.unpack( '<b', _pbuf[12:13] )[0]
                p.tag          = struct.unpack( '<b', _pbuf[13:14] )[0]
            elif( data_type == 2 ):
                p.x            = struct.unpack( '<h', _pbuf[0:2] )[0] / 1.0e2
                p.y            = struct.unpack( '<h', _pbuf[2:4] )[0] / 1.0e2
                p.z            = struct.unpack( '<h', _pbuf[4:6] )[0] / 1.0e2
                p.reflectivity = struct.unpack( '<b', _pbuf[6:7] )[0]
                p.tag          = struct.unpack( '<b', _pbuf[7:8] )[0]
            ret.append(p)

        return ret


    # frame packages
    def parse_frame_packages( self, buf : bytes ):
        ret = []

        while( len(buf) > 0 ):
            # read package header
            (_data, buf) = pop_n(27, buf)
            pkg          = Package()
            pkg.header   = self.parse_package_header(_data)

            # parse points
            (_data, buf) = pop_n(pkg.header.length, buf)
            pkg.points   = self.parse_points(_data, pkg.header.data_type)

            # add package
            ret.append(pkg)

        return ret


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

        # open binary file
        _lvx2 = open(self._in_file, 'rb')
        if( _lvx2 == None ):
            print('File Open Error. (e.g. file not exist)')
            return

        # read public header
        print('==================== Public Header ====================')
        _data = _lvx2.read(24)
        self._pub_header = self.parse_public_header(_data)
        if( self._pub_header == None ):
            return

        # read private heaer
        print('==================== Private Header ====================')
        _data = _lvx2.read(5)
        self._prv_header = self.parse_private_header(_data)
        if( self._prv_header == None ):
            return

        # read device information
        print('==================== Device Information ====================')

        for i in range(self._prv_header.device_count):
            print('Device #{}'.format(i))
            _data = _lvx2.read(63)
            _dev  = self.parse_device_information(_data)

            if( _dev == None ):
                return

            # add device
            self._devices.append(_dev)


        print('==================== Point Cloud Data ====================')
        while(True):
            # read frame data
            _data = _lvx2.read(24)
            if(_data == b''):
                print('EOF')
                break

            # parse frame header
            _frm = Frame()
            _frm.header = self.parse_frame_header(_data)

            # parse packages
            _data          = _lvx2.read( _frm.header.frame_size - 24 )
            _frm.packages  = self.parse_frame_packages(_data)

            # add timestamp
            _ts = 0.0
            for _pkg in _frm.packages:
                _ts = _ts + _pkg.header.timestamp
            _frm.timestamp = _ts / len(_frm.packages)

            # add frame
            self._frames.append(_frm)

        # close binary file
        _lvx2.close()

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
        for _frm in self._frames:
            i = i + 1
            print('processing frames ({0}/{1})'.format(i, len(self._frames)))

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
