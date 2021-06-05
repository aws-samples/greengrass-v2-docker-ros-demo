#!/usr/bin/env python3
"""
Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
SPDX-License-Identifier: MIT-0

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from functools import partial
import threading
import awsiot.greengrasscoreipc as gg
import awsiot.greengrasscoreipc.model as model

class GreengrassBridge(Node):

    ros_publishers = {}
    iot_publishers = {}

    def __init__(self):    
        super().__init__("greengrass_bridge")
        
        self.declare_parameters(
            namespace="",
            parameters=[
                ("iot_topics", []),
                ("ros_topics", []),
                ("timeout", 10)
            ]
        )
        
        self.get_logger().info("Initializing Greengrass ROS2 Bridge...")
        
        try:
            self.iot_topics = self.get_parameter("iot_topics")._value
            self.ros_topics = self.get_parameter("ros_topics")._value
        except:
            self.iot_topics = []
            self.ros_topics = []
            self.get_logger().error("Invalid ros topics / iot topics arguments.")
            
        self.get_logger().info(" ==== ROS TOPICS ====")
        self.get_logger().info(str(self.ros_topics))

        self.get_logger().info(" ==== IoT TOPICS ====")
        self.get_logger().info(str(self.iot_topics))

        self.timeout = self.get_parameter("timeout")._value
        self.get_logger().info("Timeout: %s" % self.timeout)
        
        self.ipc_client = gg.connect()
        self.init_subscribers()
        
    def init_subscribers(self):

        for topic in self.iot_topics:
            self.get_logger().info("Setting up IoT subscriber for %s" % topic)
            self.ros_publishers[topic] = self.create_publisher(String, topic, 10)
            handler = gg.client.SubscribeToIoTCoreStreamHandler()
            handler.on_stream_event = self.execute_publish_thread
            operation = self.ipc_client.new_subscribe_to_iot_core(stream_handler=handler)
            response = operation.activate(model.SubscribeToIoTCoreRequest(
                topic_name=topic.strip(),
                qos=model.QOS.AT_LEAST_ONCE
            ))
            response.result()
            self.get_logger().info("Subscribed to iot topic %s" % topic.strip())
            
        for ros_topic in self.ros_topics:
            self.get_logger().info("Setting up ROS Topic subscriber for %s" % ros_topic)
            self.iot_publishers[ros_topic] = partial(self.publish_to_iot, ros_topic)
            self.create_subscription(String, ros_topic.strip(), self.iot_publishers[ros_topic], 1)
            self.get_logger().info("Subscribed to ros topic %s" % ros_topic.strip())
    
    def execute_publish_thread(self, event: model.IoTCoreMessage):
        try:
            self.get_logger().info("Kicking off the thread.")
            t = threading.Thread(target = self.publish_to_ros2, args=[event])
            t.start()
        except Exception as ex:
            self.get_logger().error(str(ex))
        
    def publish_to_ros2(self, event: model.IoTCoreMessage):
        try:
            message = str(event.message.payload, "utf-8")
            topic = event.message.topic_name
            self.get_logger().info("Received message on topic %s from AWS IoT Core %s" % (message, topic))

            ros_msg = String()
            ros_msg.data = message
            self.ros_publishers[topic].publish(ros_msg)
            self.get_logger().info("Published message: %s to topic %s" % (message, topic))
        except Exception as ex:
            self.get_logger().error(str(ex))
            
    def publish_to_iot(self, topic, msg):
        
        self.get_logger().info("Publishing message to the cloud: %s" % msg.data)

        operation = self.ipc_client.new_publish_to_iot_core()
        operation.activate(model.PublishToIoTCoreRequest(
            topic_name=topic,
            qos=model.QOS.AT_LEAST_ONCE,
            payload=json.dumps(msg.data).encode(),
        ))
        
        future = operation.get_response()
        future.result(self.timeout)    
        
def main(args=None):

    rclpy.init(args=args)
    
    node = GreengrassBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.get_logger().info("Closing ROS Bridge")
    node.destroy_node()
        
if __name__ == "__main__":
        main()