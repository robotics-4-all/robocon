#!/usr/bin/env python

# Copyright (C) 2020 Panayiotou, Konstantinos <klpanagi@gmail.com>
# Author: Panayiotou, Konstantinos <klpanagi@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.

from typing import Any
import rospy

from commlib.endpoints import endpoint_factory, EndpointType, TransportType
from ros_msg_transform import ros_msg_to_dict, dict_to_ros_msg


class B2RTopicBridge:

    def __init__(self, ros_topic: str, msg_type: Any, broker_type: Any, broker_uri, broker_conn_params):
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params
        self.queue_size = 10

        self._init_ros_endpoint()
        self._init_broker_endpoint()
        print(f'Started B2R Topic Bridge: {broker_type.name.lower()}://{broker_uri} -> ros://{self.ros_topic}')

    def _init_ros_endpoint(self):
        self.ros_pub = rospy.Publisher(
            self.ros_topic,
            self.msg_type,
            queue_size=self.queue_size
        )

    def _init_broker_endpoint(self):
        self.bsub = endpoint_factory(EndpointType.Subscriber,
            self.broker_type)(
            topic=self.broker_uri,
            on_message=self.on_msg,
            conn_params=self.broker_conn_params
        )
        self.bsub.run()

    def on_msg(self, data):
        _msg = dict_to_ros_msg(data, self.msg_type)
        rospy.loginfo(f'Publishing: {_msg}')
        self.ros_pub.publish(_msg)


class R2BTopicBridge:

    def __init__(self, ros_topic: str, msg_type: Any,
                 broker_type: Any, broker_uri,
                 broker_conn_params):
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_broker_endpoint()
        self._init_ros_endpoint()
        print(f'Started R2B Topic Bridge: ros://{self.ros_topic} -> {broker_type.name.lower()}://{broker_uri}')

    def _init_ros_endpoint(self):
        self.ros_sub = rospy.Subscriber(
            self.ros_topic,
            self.msg_type,
            self.on_msg
        )

    def _init_broker_endpoint(self):
        self.bpub = endpoint_factory(EndpointType.Publisher,
            self.broker_type)(
            topic=self.broker_uri,
            conn_params=self.broker_conn_params
        )
        self.bpub.run()

    def on_msg(self, msg):
        _data = ros_msg_to_dict(msg)
        rospy.loginfo(f'Publishing: {_data}')
        self.bpub.publish(_data)

class MyROSRobotBridgeNode:
    def __init__(self):
        rospy.init_node('MyROSRobot_bridge')
        self.br_list = []
        self._init_broker()
        self._init_bridges()

    def _init_broker(self):
        ## Broker Connection for Bridge ------------------------------------------>
        self.broker_type = TransportType.MQTT
        from commlib.transports.mqtt import ConnectionParameters
        self.conn_params = ConnectionParameters(
            host='localhost',
            port=1883,
            username='',
            password='',
            ssl=False
        )

    def _init_bridges(self):
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from nav_msgs.msg import Odometry
        br = R2BTopicBridge('/odom', Odometry, self.broker_type, 'robot/odom', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import Range
        br = R2BTopicBridge('/sonar/front_left', Range, self.broker_type, 'robot/sonar_fl', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import Range
        br = R2BTopicBridge('/sonar/front_right', Range, self.broker_type, 'robot/sonar_fr', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import Range
        br = R2BTopicBridge('/sonar/rear', Range, self.broker_type, 'robot/sonar_rear', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge B2R
        from geometry_msgs.msg import Twist
        br = B2RTopicBridge('/cmd_vel', Twist, self.broker_type, 'robot/cmd_vel', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge B2R
        from geometry_msgs.msg import Twist
        br = B2RTopicBridge('/motor_power', Twist, self.broker_type, 'robot/motor_power', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from std_srvs.srv import Empty
        br = B2RServiceBridge('/reset', Empty, self.broker_type, 'robot/reset', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    node = MyROSRobotBridgeNode()
    node.run()


if __name__ == "__main__":
    main()