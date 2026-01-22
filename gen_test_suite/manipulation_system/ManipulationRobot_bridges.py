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

class ManipulationRobotBridgeNode(Node):
    def __init__(self):
        super().__init__('ManipulationRobot_bridge')
        self.br_list = []
        self._init_broker()
        self._init_bridges()

    def _init_broker(self):
        ## Broker Connection for Bridge ------------------------------------------>
        self.broker_type = TransportType.MQTT
        from commlib.transports.mqtt import ConnectionParameters
        self.conn_params = ConnectionParameters(
            host='192.168.1.50',
            port=1883,
            username='',
            password='',
            ssl=False
        )

    def _init_bridges(self):
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from moveit_msgs.msg import DisplayTrajectory
        br = R2BTopicBridge(self, '/display_planned_path', DisplayTrajectory, self.broker_type, 'arm/move_group/display_planned_path', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge B2R
        from sensor_msgs.msg import JointState
        br = B2RTopicBridge(self, '/joint_states', JointState, self.broker_type, 'arm/move_group/joint_states', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## Topic Bridge B2R
        from std_msgs.msg import String
        br = B2RTopicBridge(self, '/robot_description', String, self.broker_type, 'arm/move_group/robot_description', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from moveit_msgs.srv import GetPlanningScene
        br = B2RServiceBridge(self, '/get_planning_scene', GetPlanningScene, self.broker_type, 'arm/move_group/get_planning_scene', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## RPC Bridge B2R
        from moveit_msgs.srv import GetPositionIK
        br = B2RServiceBridge(self, '/compute_ik', GetPositionIK, self.broker_type, 'arm/move_group/compute_ik', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------
        ## <-----------------------------------------------------------------------
        ## <-----------------------------------------------------------------------
        ## Topic Bridge R2B
        from sensor_msgs.msg import JointState
        br = R2BTopicBridge(self, '/joint_states', JointState, self.broker_type, 'arm/joint_states', self.conn_params)
        self.br_list.append(br)
        ## <-----------------------------------------------------------------------


def main():
    rclpy.init()
    node = ManipulationRobotBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()