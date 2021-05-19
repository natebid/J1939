#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from std_msgs.msg import UInt8, String 
from std_msgs.msg import Float64
from j1939_ros_interface.msg import J1939
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import Quaternion, Vector3
from message_filters import ApproximateTimeSynchronizer 
import struct


pub = rospy.Publisher('/converted/imu_data', Imu, queue_size=1)

def callback(accel, angular, slope):

    ### acceleration data
    accel_array = bytearray(accel.data)  

    hex0_a = format(accel_array[0], '02x')
    hex1_a = format(accel_array[1], '02x')
    hex2_a = format(accel_array[2], '02x')
    hex3_a = format(accel_array[3], '02x')    
    hex4_a = format(accel_array[4], '02x')
    hex5_a = format(accel_array[5], '02x') 

    hex_ax = hex3_a + hex2_a
    hex_ay = hex1_a + hex0_a
    hex_az = hex5_a + hex4_a

    ax = (((int(hex_ax, 16))/100)-320)
    ay = (((int(hex_ay, 16))/100)-320)
    az = (((int(hex_az, 16))/100)-320)

    # print ('%.4f' % ax)
    # print ('%.4f' % ay)
    # print ('%.4f' % az)  

    ### orientation data
    slope_array = bytearray(slope.data)

    hex0_s = format(slope_array[0], '02x')
    hex1_s = format(slope_array[1], '02x')
    hex2_s = format(slope_array[2], '02x')
    hex3_s = format(slope_array[3], '02x')
    hex4_s = format(slope_array[4], '02x')
    hex5_s = format(slope_array[5], '02x')

    hex_sx = hex2_s + hex1_s + hex0_s 
    hex_sy = hex5_s + hex4_s + hex3_s

    sx =  (((int(hex_sx, 16))/32768)-250)  ##pitch
    sy =  (((int(hex_sy, 16))/32768)-250)  ##roll

    # print ('%.4f' % sx)
    # print ('%.4f' % sy)

    ### angular rate data
    angular_array = bytearray(angular.data)  

    hex0_v = format(angular_array[0], '02x')
    hex1_v = format(angular_array[1], '02x')
    hex2_v = format(angular_array[2], '02x')
    hex3_v = format(angular_array[3], '02x')    
    hex4_v = format(angular_array[4], '02x')
    hex5_v = format(angular_array[5], '02x') 

    hex_vx = hex1_v + hex0_v
    hex_vy = hex3_v + hex2_v
    hex_vz = hex5_v + hex4_v

    vx = (((int(hex_vx, 16))/128)-250)
    vy = (((int(hex_vy, 16))/128)-250)
    vz = (((int(hex_vz, 16))/128)-250)

    # print ('%.4f' % vx)
    # print ('%.4f' % vy)
    # print ('%.4f' % vz)  

    # use similar to msg.data i.e accel.data, angular.data, slope.data
    # data_bytes = bytearray(accel.data)
    # print data_bytes[0].to_bytes(2, 'big') 
    # ax = pub.publish(int(data_bytes[0]))

    # data_bytes = bytearray(angular.data)
    # vx = pub.publish(int(data_bytes[0]))    


    imu_message = Imu()

    j1939_orientation = Quaternion()

    # Calculate a quaternion representing the orientation
    accel = ax, ay, az
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    j1939_orientation.x = sx
    j1939_orientation.y = sy
    # j1939_orientation.z = *USE DATA CONVERTED FROM slope.data*
    # j1939_orientation.w = *USE DATA CONVERTED FROM slope.data*

    j1939_angular = Vector3()

    j1939_angular.x = vx
    j1939_angular.y = vy
    j1939_angular.z = vz
    
    j1939_accel = Vector3()

    j1939_accel.x = ax
    j1939_accel.y = ay
    j1939_accel.z = az

    # Load up the IMU message
    o = imu_message.orientation
    o.x, o.y, o.z, o.w = orientation


    # imu_message.orientation = j1939_orientation

    imu_message.angular_velocity = j1939_angular

    imu_message.linear_acceleration = j1939_accel


    imu_message.header.stamp = rospy.Time.now()
    imu_message.header.frame_id = "base"
    imu_message.header.seq = "seq"

    # imu_message.orientation_covariance = -1

    # imu_message.angular_velocity_covariance = 1
    
    # imu_message.linear_acceleration_covariance = 1

    pub.publish(imu_message)


def receiver_node():

    rospy.init_node('receiver_node', anonymous=True)

    accel_sub = message_filters.Subscriber('/j1939_1/imu_accel', J1939) #verify message type

    angular_sub = message_filters.Subscriber('/j1939_1/imu_angular', J1939)

    slope_sub = message_filters.Subscriber('/j1939_1/imu_slope', J1939)

    tss = message_filters.ApproximateTimeSynchronizer([accel_sub, angular_sub, slope_sub], 10, 0.1, allow_headerless=True)

    tss.registerCallback(callback)
    

    rospy.spin()

if __name__ == '__main__':
    receiver_node()


