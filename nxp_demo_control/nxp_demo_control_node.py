#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from tflite_msgs.msg import TFLite, TFInference
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class NXPDemoControl(Node):
    def __init__(self):
        super().__init__('nxp_demo_control_node')


        command_input_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Twist command output topic name.')

        command_output_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Twist command input topic name.')

        tf_lite_topic_0_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='TFLite_0 returns topic.')

        
        tf_lite_topic_1_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='TFLite_1 returns topic.')

        threshold_0_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Threshold_0 value for inference score.')

        threshold_1_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Threshold_1 value for inference score.')



        self.declare_parameter("cmd_input_topic", "/requested_vel", 
            command_input_topic_descriptor)

        self.declare_parameter("cmd_output_topic", "/cmd_vel", 
            command_output_topic_descriptor)

        self.declare_parameter("tflite_topic_0", "/TFLiteSim", 
            tf_lite_0_topic_descriptor)

        self.declare_parameter("tflite_topic_1", "/TFLiteReal", 
            tf_lite_1_topic_descriptor)

        self.declare_parameter("threshold_0", 0.5, 
            threshold_0_descriptor)

        self.declare_parameter("threshold_1", 0.5, 
            threshold_1_descriptor)

        self.TFLiteTopic0 = self.get_parameter("tflite_topic_0").value
        self.TFLiteTopic1 = self.get_parameter("tflite_topic_1").value
        self.Threshold0 = float(self.get_parameter("threshold_0").value)
        self.Threshold1 = float(self.get_parameter("threshold_1").value)
        self.CmdSubTopic = self.get_parameter("cmd_input_topic").value
        self.CmdPubTopic = self.get_parameter("cmd_output_topic").value

        self.TFLiteSub0 = self.create_subscription(TFLite, '{:s}'.format(self.TFLiteTopic0), self.TFLiteCallback0, 1)
        self.TFLiteSub1 = self.create_subscription(TFLite, '{:s}'.format(self.TFLiteTopic1), self.TFLiteCallback1, 1)
        self.CmdSub = self.create_subscription(Twist, '{:s}'.format(self.CmdSubTopic), self.CMDCallback, 1)
        self.CmdPub = self.create_publisher(Twist, '{:s}'.format(self.CmdPubTopic), 0)
        self.StopMotors0 = False
        self.StopMotors1 = False

    def CMDCallback(self, msgRequest):
        linearX = msgRequest.linear.x
        angularZ = msgRequest.angular.z
        msgCMD = Twist()
        if self.StopMotors0 or self.StopMotors1: 
            linearX = 0.0
            angularZ = 0.0
        msgCMD.linear.x = linearX
        msgCMD.linear.y = 0.0
        msgCMD.linear.z = 0.0
        msgCMD.angular.x = 0.0
        msgCMD.angular.y = 0.0
        msgCMD.angular.z = angularZ
        self.CmdPub.publish(msgCMD)
        return

    def TFLiteCallback0(self, msgTFLite):
        for inference in msgTFLite.inference:
            if inference.score >= self.Threshold0:
                if inference.label.upper() == "PERSON":
                    self.StopMotors0 = True
                    return
        self.StopMotors0 = False
        return

    def TFLiteCallback1(self, msgTFLite):
        for inference in msgTFLite.inference:
            if inference.score >= self.Threshold1:
                if inference.label.upper() == "PERSON":
                    self.StopMotors1 = True
                    return
        self.StopMotors1 = False
        return

if __name__ == '__main__':
    rclpy.init()
    NXPDC = NXPDemoControl()
    rclpy.spin(NXPDC)
    NXPDC.destroy_node()
    rclpy.shutdown()
