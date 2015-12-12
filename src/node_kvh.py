#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Heriot-Watt University nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Original authors:
#   Valerio De Carolis, Marian Andrecki, Corina Barbalata, Gordon Frost

import os
import argparse
import traceback

import serial
from serial import Serial, SerialException

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('kvh_gyroscope')

import tf
import tf.transformations as tft

from sensor_msgs.msg import Imu


# default serial configuration
DEFAULT_REPEAT = 5
DEFAULT_CONF = {
    'port': '/vdev/tty_gyro',
    'baudrate': 38400,
    'bytesize': serial.EIGHTBITS,
    'parity': serial.PARITY_NONE,
    'stopbits': serial.STOPBITS_ONE,
    'timeout': 1
}

# kvh related
KVH_MODE_R = 'R'            # output rate
KVH_MODE_A = 'A'            # incremental angle
KVH_MODE_P = 'P'            # integrated angle
KVH_CMD_ZERO = 'Z'          # zero the integrated angle
KVH_CMD_DELAY = 0.5         # default delay after sending a command
KVH_RATE = 100.0            # output frequency of gyroscope (Hz)
KVH_OFFSET = 0.005          # bias offset (from datasheet) (deg/sec) (shown as 20deg/hr and in the test check)
KVH_SIGMA = 0.0015          # output error (from datasheet) (deg/sec) (shown as 1500 ppm at one sigma)

# default config
DEFAULT_LATITUDE = 55.912       # deg   (default Edinburgh)
DEFAULT_RATE = 10.0             # Hz
DEFAULT_FRAME = 'base_link'
TOPIC_GYRO = 'gyro/orientation'


def wrap_360(angle):
    while angle < 0:
        angle += 360

    while angle >= 360:
        angle -= 360

    return angle


class GyroscopeNode(object):

    def __init__(self, name, **kwargs):
        self.name = name
        self.ser = None
        self.connected = False
        self.serconf = dict()

        # serial config
        self.serconf.update(DEFAULT_CONF)
        self.serconf['port'] = rospy.get_param('~port', DEFAULT_CONF['port'])
        self.serconf['baudrate'] = rospy.get_param('~baud', DEFAULT_CONF['baudrate'])

        # internal state
        self.frame_id = rospy.get_param('~frame', DEFAULT_FRAME)
        self.node_rate = rospy.get_param('~rate', DEFAULT_RATE)
        self.mode = rospy.get_param('~mode', KVH_MODE_R)

        self.latitude_deg = float(rospy.get_param('~latitude', DEFAULT_LATITUDE))
        self.latitude = np.deg2rad(self.latitude_deg)
        rospy.logwarn('%s: please note that the gyro latitude is %s deg', self.name, self.latitude_deg)

        self.zero_start = bool(kwargs.get('zero_start', False))
        # self.offset_yaw = float(kwargs.get('offset', 0.0))

        # measurements
        self.yaw = 0.0
        self.yaw_dot = 0.0
        self.yaw_std_z = 0.0
        self.valid = False
        self.data = None

        # from kvh manual (deg/s)
        self.earth_rate = -15.04107 * np.sin(self.latitude) / 3600.0

        # ros interface
        self.pub_gyro = rospy.Publisher(TOPIC_GYRO, Imu, queue_size=10, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_rate), self.send_message)

        # test mode
        self.test_enabled = bool(kwargs.get('test_enabled', False))
        self.test_output = kwargs.get('test_output', 'gyro.csv')
        self.test_file = None

        if self.test_enabled:
            self.test_file = open(self.test_output, 'wt')
            self.test_file.write('time,raw,bit,rate,yaw\n')


    def handle_reading(self, data, valid):
        self.valid = valid
        self.data = data

        if not self.valid:
            rospy.logerr('%s: got reading but device is signalling a failure (BIT): %s %s', self.name, data, self.valid)
            return

        if self.mode == KVH_MODE_R:
            # gyro is sending the rate [deg/s]
            self.yaw_dot = data + self.earth_rate
        elif self.mode == KVH_MODE_A:
            # gyro is sending the angular difference [deg]
            self.yaw_dot = data * KVH_RATE + self.earth_rate

            # the yaw is calculated by integration
            self.yaw += data + (self.earth_rate / KVH_RATE)
            self.yaw = wrap_360(self.yaw)

            # update covariance
            self.yaw_std_z += KVH_OFFSET / KVH_RATE
        elif self.mode == KVH_MODE_P:
            pass
        else:
            pass

        if self.test_enabled:
            line = '{0},{1},{2},{3},{4}\n'.format(rospy.Time.now(), self.data, self.valid, self.yaw_dot, self.yaw)
            self.test_file.write(line)


    def send_message(self, event=None):
        if not self.valid or not self.connected:
            rospy.loginfo('%s: measurements not valid, not sending imu message!', self.name)
            return

        # NOTE:
        #   the axis convention is taken from the old driver implementation
        #   this should be checked again and explicitly stated in the documentation
        rotation = tft.quaternion_from_euler(0, 0, np.deg2rad(-self.yaw), axes='sxyz')

        # NOTE:
        # This is a message to hold data from an IMU (Inertial Measurement Unit)
        #
        # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
        #
        # If the covariance of the measurement is known, it should be filled in (if all you know is the
        # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
        # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
        # data a covariance will have to be assumed or gotten from some other source
        #
        # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation
        # estimate), please set element 0 of the associated covariance matrix to -1
        # If you are interpreting this message, please check for a value of -1 in the first element of each
        # covariance matrix, and disregard the associated estimate.
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = rotation[0]
        msg.orientation.y = rotation[1]
        msg.orientation.z = rotation[2]
        msg.orientation.w = rotation[3]

        # this is derived by looking at the datasheet under the bias offset factor
        # and it should include the drift of the gyro if the yaw is computed by integration
        msg.orientation_covariance[8] = np.deg2rad(self.yaw_std_z**2)

        # this is to notify that this imu is not measuring this data
        msg.linear_acceleration_covariance[0] = -1

        msg.angular_velocity.z = np.deg2rad(-self.yaw_dot)

        # this is derived by looking at the datasheet
        msg.angular_velocity_covariance[8] = np.deg2rad(KVH_SIGMA ** 2)

        self.pub_gyro.publish(msg)

        if self.mode == KVH_MODE_R:
            rospy.loginfo('%s: raw data: %+.5f -- yaw rate: %+.5f', self.name, self.data, self.yaw_dot)
        else:
            rospy.loginfo('%s: raw data: %+.5f -- yaw rate: %+.5f -- yaw: %+.5f', self.name, self.data, self.yaw_dot, self.yaw)


    def send_cmd(self, cmd=None):
        if not self.connected or cmd == None:
            return

        for n in xrange(DEFAULT_REPEAT):
            self.ser.write(cmd)

        rospy.sleep(KVH_CMD_DELAY)


    def run(self):
        # connection main loop
        while not rospy.is_shutdown():
            try:
                self.ser = Serial(**self.serconf)
            except SerialException:
                rospy.logerr('%s: device not found, waiting for device ...', self.name)
            except ValueError:
                rospy.logfatal('%s: bad port configuration!', self.name)
                rospy.signal_shutdown('bad config')
                break
            else:
                rospy.loginfo('%s: found device, reading serial ...', self.name)
                self.connected = True

            # mode setting
            if self.connected:
                rospy.loginfo('%s: setting kvh mode: %s', self.name, self.mode)
                self.send_cmd(self.mode)

                if self.mode == KVH_MODE_P and self.zero_start:
                    rospy.loginfo('%s: zeroing integration', self.name)
                    self.send_cmd(KVH_CMD_ZERO)

                rospy.loginfo('%s: starting data parsing', self.name)

            # data processing loop
            while self.connected:
                try:
                    line = self.ser.readline()
                except SerialException:
                    self.connected = False
                    rospy.logerr('%s: connection lost, waiting for device ...', self.name)
                    break
                except Exception as e:
                    self.connected = False
                    rospy.logwarn('%s: uncaught exception: %s', self.name, e)
                    rospy.signal_shutdown('uncaught exception')
                    break

                if len(line) != 0:
                    msg = line.strip()           # remove any return carriage
                    items = msg.split()          # separate messages

                    if len(items) == 2:
                        try:
                            raw_reading = float(items[0])
                            bit_reading = bool(items[1])        # built-in self-test (0 -> not good, 1 -> fog ok)
                        except Exception:
                            rospy.logerr('%s: received bad data:\n%s', self.name, traceback.format_exc())
                            continue

                        self.handle_reading(raw_reading, bit_reading)
                else:
                    # ignoring data
                    pass

            # handle disconnection
            if self.ser is not None and self.ser.isOpen():
                self.ser.close()

            # wait before reconnect
            rospy.sleep(1)

        # close connection
        if self.ser is not None and self.ser.isOpen():
            self.ser.close()


def main():
    parser = argparse.ArgumentParser(
        description='Driver for KVH Fiber Optic Gyroscope (DSP-3000).',
        epilog='This is part of vehicle_driver module.'
    )

    # option group
    parser.add_argument('--test', action='store_true', help='Start the driver in test mode (recording data to disk).')
    parser.add_argument('--output', default='gyro.csv', help='Output file to write during the test mode.')
    parser.add_argument('--verbose', action='store_true', help='Print detailed information.')

    # init
    rospy.init_node('kvh_gyroscope')
    name = rospy.get_name()

    rospy.loginfo('%s initializing ...', name)

    # parse command line
    ros_args = rospy.myargv()
    ros_args.pop(0)
    args = parser.parse_args(ros_args)

    # handle test
    test_enabled = args.test
    test_output = args.output

    if test_enabled:
        rospy.logwarn('%s: starting driver in test mode: %s', name, test_output)

        if os.path.exists(test_output):
            rospy.logwarn('%s: removing previous measurements: %s', name, test_output)
            os.remove(test_output)
        else:
            with open(test_output, 'a'):
                os.utime(test_output, None)

    # load config
    config = {
        'test_enabled': test_enabled,
        'test_output': test_output
    }

    # start client
    rospy.loginfo('%s: gyro config: %s', name, config)

    gn = GyroscopeNode(name, **config)

    def handle_close():
        if gn.test_file is not None:
            rospy.logwarn('%s: closing measurements file: %s', name, test_output)
            gn.test_file.flush()
            gn.test_file.close()

    rospy.on_shutdown(handle_close)

    # start measurements
    gn.run()

    # graceful shutdown
    rospy.loginfo('%s shutdown complete!', name)


if __name__ == '__main__':
    main()
