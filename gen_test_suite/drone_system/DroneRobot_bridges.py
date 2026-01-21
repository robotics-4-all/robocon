#!/usr/bin/env python3

from typing import Any
import rclpy
from rclpy.node import Node

from commlib.endpoints import endpoint_factory, EndpointType, TransportType
from ros2_msg_transform import (
    ros2_msg_to_dict, dict_to_ros2_msg_from_ns, dict_to_ros2_msg,
    dict_to_ros2_srv_from_ns
)


class B2RServiceBridge:

    def __init__(self, nh, ros_uri: str, msg_type: Any, broker_type: Any, broker_uri, broker_conn_params):
        self.nh = nh
        self.ros_uri = ros_uri
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.ros_client = self.nh.create_client(self.msg_type, self.ros_uri)
        tcount = 1.0
        max_tcount = 10.0
        while not self.ros_client.wait_for_service(timeout_sec=tcount):
            self.nh.get_logger().info(
                'ROS Service not available, waiting again...')
            if tcount > max_tcount:
                print('[ERROR] - ROS Service connection timeout!')
                break
            tcount += 1.0

    def _init_broker_endpoint(self):
        self.bservice = endpoint_factory(EndpointType.RPCService,
            self.broker_type)(
            rpc_name=self.broker_uri,
            on_request=self.on_request,
            conn_params=self.broker_conn_params
        )
        self.bservice.run()

    def on_request(self, data):
        _req = dict_to_ros2_msg(data, self.msg_type.Request)
        self.nh.get_logger().info(f'Calling Service: {self.ros_uri}')
        future = self.send_request(_req)
        resp = self.wait_for_ros_resp(future)
        _dresp = ros2_msg_to_dict(resp)
        return _dresp

    def send_request(self, req):
        future = self.ros_client.call_async(req)
        rclpy.spin_until_future_complete(self.nh, future)
        return future.result()


class B2RTopicBridge:

    def __init__(self, nh, ros_topic: str, msg_type: Any, broker_type: Any, broker_uri, broker_conn_params):
        self.nh = nh
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.ros_pub = self.nh.create_publisher(self.msg_type, self.ros_topic, 10)

    def _init_broker_endpoint(self):
        self.bsub = endpoint_factory(EndpointType.Subscriber,
            self.broker_type)(
            topic=self.broker_uri,
            on_message=self.on_msg,
            conn_params=self.broker_conn_params
        )
        self.bsub.run()

    def on_msg(self, data):
        _msg = dict_to_ros2_msg(data, self.msg_type)
        self.nh.get_logger().info('Publishing: "%s"' % _msg)
        self.ros_pub.publish(_msg)


class R2BTopicBridge:

    def __init__(self, nh, ros_topic: str, msg_type: Any, broker_type: Any, broker_uri, broker_conn_params):
        self.nh = nh
        self.ros_topic = ros_topic
        self.broker_uri = broker_uri
        self.msg_type = msg_type
        self.broker_type = broker_type
        self.broker_conn_params = broker_conn_params

        self._init_ros_endpoint()
        self._init_broker_endpoint()

    def _init_ros_endpoint(self):
        self.nh.ros_sub = self.nh.create_subscription(
            self.msg_type,
            self.ros_topic,
            self.on_msg,
            10)

    def _init_broker_endpoint(self):
        self.bpub = endpoint_factory(EndpointType.Publisher,
            self.broker_type)(
            topic=self.broker_uri,
            conn_params=self.broker_conn_params
        )

    def on_msg(self, msg):
        _data = ros2_msg_to_dict(msg)
        self.nh.get_logger().info('Publishing: "%s"' % _data)
        self.bpub.publish(_data)

class DroneRobotBridgeNode(Node):
    def __init__(self):
        super().__init__('DroneRobot_bridge')
        self.br_list = []
        self._init_broker()
        self._init_bridges()

    def _init_broker(self):
        ## Broker Connection for Bridge ------------------------------------------>
        self.broker_type = TransportType.REDIS
        from commlib.transports.redis import ConnectionParameters
        self.conn_params = ConnectionParameters(
            host='ground-station.local',
            port=int(6379),
            db=0,
            username='',
            password='',
            ssl=False
        )

    def _init_bridges(self):
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from geometry_msgs.msg import PoseStamped
        br = R2BTopicBridge(self, '/mavros/local_position/pose', PoseStamped, self.broker_type, 'drone1/local_pose', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import NavSatFix
        br = R2BTopicBridge(self, '/mavros/global_position/global', NavSatFix, self.broker_type, 'drone1/global_pose', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import BatteryState
        br = R2BTopicBridge(self, '/mavros/battery', BatteryState, self.broker_type, 'drone1/battery', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from mavros_msgs.msg import State
        br = R2BTopicBridge(self, '/mavros/state', State, self.broker_type, 'drone1/state', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import Imu
        br = R2BTopicBridge(self, '/mavros/imu/data', Imu, self.broker_type, 'drone1/imu', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from mavros_msgs.srv import CommandBool
        br = B2RServiceBridge(self, '/mavros/cmd/arming', CommandBool, self.broker_type, 'drone1/arm', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from mavros_msgs.srv import SetMode
        br = B2RServiceBridge(self, '/mavros/set_mode', SetMode, self.broker_type, 'drone1/set_mode', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from mavros_msgs.srv import CommandTOL
        br = B2RServiceBridge(self, '/mavros/cmd/takeoff', CommandTOL, self.broker_type, 'drone1/takeoff', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from mavros_msgs.srv import CommandTOL
        br = B2RServiceBridge(self, '/mavros/cmd/land', CommandTOL, self.broker_type, 'drone1/land', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------


def main():
    rclpy.init()
    node = DroneRobotBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()