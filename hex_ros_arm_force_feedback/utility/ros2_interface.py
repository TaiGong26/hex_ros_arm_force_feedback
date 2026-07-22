#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import threading

import numpy as np
import rclpy
import rclpy.node

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboArmCtrl,
    HexRosRoboGripCtrl,
    HexRosRoboManipCtrl,
    HexRosRoboManipCtrlStamped,
    HexRosRoboManipStateStamped,
    HexRosTeleopKeyboardStateStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntState,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmState,
    HexDcRoboGripCtrl,
    HexDcRoboGripState,
    HexDcRoboManipCtrl,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import HexDcTeleopKeyboardState

from .interface_base import InterfaceBase

_LETTERS = [chr(c) for c in range(ord('a'), ord('z') + 1)]


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 500.0)
        self._rate_param["ros"] = self.__node.get_parameter('rate_ros').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### parameters
        self.__node.declare_parameter('rate_teleop', 100.0)
        self.__node.declare_parameter('model_urdf', "")
        self.__node.declare_parameter('model_frame_id', "base_link")
        self.__node.declare_parameter(
            'pose_end_in_flange',
            [0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        )
        self.__node.declare_parameter('gravity', [0.0, 0.0, -9.81])
        self.__node.declare_parameter('arm_start_pos',
                                      [0.0, -1.5, 3.0, 0.07, 0.0, 0.0])
        self.__node.declare_parameter('arm_end_pos',
                                      [0.0, -1.5, 3.0, 0.07, 0.0, 0.0])
        self.__node.declare_parameter('grip_stable_pos', [0.5])
        self.__node.declare_parameter('arm_stable_kp',
                                      [200.0, 200.0, 250.0, 150.0, 100.0, 100.0])
        self.__node.declare_parameter('arm_stable_kd', [5.0, 5.0, 5.0, 5.0, 2.0, 2.0])
        self.__node.declare_parameter('grip_stable_kp', [10.0])
        self.__node.declare_parameter('grip_stable_kd', [0.5])
        self.__node.declare_parameter('arm_impedance_kp',
                                      [100.0, 100.0, 125.0, 75.0, 50.0, 50.0])
        self.__node.declare_parameter('arm_impedance_kd',
                                      [2.5, 2.5, 2.5, 2.5, 1.0, 1.0])
        self.__node.declare_parameter('arm_pos_threshold', 0.1)
        self.__node.declare_parameter('grip_impedance_kp', [10.0])
        self.__node.declare_parameter('grip_impedance_kd', [0.5])
        self.__node.declare_parameter('arrive_threshold', 0.06)

        self._rate_param.update({
            "teleop":
            self.__node.get_parameter('rate_teleop').value,
        })
        self._model_param = {
            "urdf":
            self.__node.get_parameter('model_urdf').value,
            "frame_id":
            self.__node.get_parameter('model_frame_id').value,
            "pose_end_in_flange":
            list(self.__node.get_parameter('pose_end_in_flange').value),
        }
        self._force_feedback_param = {
            "gravity":
            list(self.__node.get_parameter('gravity').value),
            "arm_start_pos":
            list(self.__node.get_parameter('arm_start_pos').value),
            "arm_end_pos":
            list(self.__node.get_parameter('arm_end_pos').value),
            "grip_stable_pos":
            list(self.__node.get_parameter('grip_stable_pos').value),
            "arm_stable_kp":
            list(self.__node.get_parameter('arm_stable_kp').value),
            "arm_stable_kd":
            list(self.__node.get_parameter('arm_stable_kd').value),
            "grip_stable_kp":
            list(self.__node.get_parameter('grip_stable_kp').value),
            "grip_stable_kd":
            list(self.__node.get_parameter('grip_stable_kd').value),
            "arm_impedance_kp":
            list(self.__node.get_parameter('arm_impedance_kp').value),
            "arm_impedance_kd":
            list(self.__node.get_parameter('arm_impedance_kd').value),
            "arm_pos_threshold":
            self.__node.get_parameter('arm_pos_threshold').value,
            "grip_impedance_kp":
            list(self.__node.get_parameter('grip_impedance_kp').value),
            "grip_impedance_kd":
            list(self.__node.get_parameter('grip_impedance_kd').value),
            "arrive_threshold":
            self.__node.get_parameter('arrive_threshold').value,
        }

        ### publisher
        self.__master_manip_ctrl_pub = self.__node.create_publisher(
            HexRosRoboManipCtrlStamped,
            'master/manip_ctrl',
            10,
        )
        self.__slave_manip_ctrl_pub = self.__node.create_publisher(
            HexRosRoboManipCtrlStamped,
            'slave/manip_ctrl',
            10,
        )

        ### subscriber
        self.__manip_state_sub = self.__node.create_subscription(
            HexRosRoboManipStateStamped,
            'manip_state',
            self.__manip_state_callback,
            10,
        )
        self.__keyboard_sub = self.__node.create_subscription(
            HexRosTeleopKeyboardStateStamped,
            'teleop_keyboard_state',
            self.__keyboard_callback,
            10,
        )
        self.__manip_state_sub
        self.__keyboard_sub

        ### master/slave subscriber
        self.__master_manip_state_sub = self.__node.create_subscription(
            HexRosRoboManipStateStamped,
            'master/manip_state',
            self.__master_manip_state_callback,
            10,
        )
        self.__slave_manip_state_sub = self.__node.create_subscription(
            HexRosRoboManipStateStamped,
            'slave/manip_state',
            self.__slave_manip_state_callback,
            10,
        )
        self.__master_manip_state_sub
        self.__slave_manip_state_sub

        ### spin thread
        self.__shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        if self.__shutting_down:
            return
        self.__shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################

    def pub_master_manip_ctrl(self, out: HexDcRoboManipCtrl):
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.manip_ctrl = HexRosRoboManipCtrl(
            arm_ctrl=self.__arm_ctrl_to_msg(out.arm_ctrl),
            grip_ctrl=self.__grip_ctrl_to_msg(out.grip_ctrl),
        )
        self.__master_manip_ctrl_pub.publish(msg)

    def pub_slave_manip_ctrl(self, out: HexDcRoboManipCtrl):
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.manip_ctrl = HexRosRoboManipCtrl(
            arm_ctrl=self.__arm_ctrl_to_msg(out.arm_ctrl),
            grip_ctrl=self.__grip_ctrl_to_msg(out.grip_ctrl),
        )
        self.__slave_manip_ctrl_pub.publish(msg)

    @staticmethod
    def __jnt_to_msg(jnt) -> HexRosJnt:
        return HexRosJnt(
            pos=np.asarray(jnt.pos, dtype=np.float64).tolist(),
            vel=np.asarray(jnt.vel, dtype=np.float64).tolist(),
            eff=np.asarray(jnt.eff, dtype=np.float64).tolist(),
            kp=np.asarray(jnt.kp, dtype=np.float64).tolist(),
            kd=np.asarray(jnt.kd, dtype=np.float64).tolist(),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64).tolist(),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64).tolist(),
        )

    @staticmethod
    def __arm_ctrl_to_msg(arm: HexDcRoboArmCtrl) -> HexRosRoboArmCtrl:
        return HexRosRoboArmCtrl(
            ctrl_mode=int(arm.ctrl_mode),
            grav=Vector3(x=arm.grav.x, y=arm.grav.y, z=arm.grav.z),
            jnt=DataInterface.__jnt_to_msg(arm.jnt),
            pose=Pose(
                position=Point(
                    x=arm.pose.position.x,
                    y=arm.pose.position.y,
                    z=arm.pose.position.z,
                ),
                orientation=Quaternion(
                    x=arm.pose.orientation.x,
                    y=arm.pose.orientation.y,
                    z=arm.pose.orientation.z,
                    w=arm.pose.orientation.w,
                ),
            ),
        )

    @staticmethod
    def __grip_ctrl_to_msg(grip: HexDcRoboGripCtrl) -> HexRosRoboGripCtrl:
        return HexRosRoboGripCtrl(
            ctrl_mode=int(grip.ctrl_mode),
            jnt=DataInterface.__jnt_to_msg(grip.jnt),
        )

    ####################
    ### subscribers
    ####################
    def __manip_state_callback(self, msg: HexRosRoboManipStateStamped):
        self._manip_state_deque.append(self.__manip_state_msg_to_dc(msg))

    def __keyboard_callback(self, msg: HexRosTeleopKeyboardStateStamped):
        self._keyboard_deque.append(self.__keyboard_msg_to_dc(msg))

    ####################
    ### master/slave callbacks
    ####################
    def __master_manip_state_callback(self, msg: HexRosRoboManipStateStamped):
        self._master_manip_state_deque.append(
            self.__manip_state_msg_to_dc(msg))

    def __slave_manip_state_callback(self, msg: HexRosRoboManipStateStamped):
        self._slave_manip_state_deque.append(
            self.__manip_state_msg_to_dc(msg))

    @staticmethod
    def __keyboard_msg_to_dc(
            msg: HexRosTeleopKeyboardStateStamped) -> HexDcTeleopKeyboardState:
        kb = msg.keyboard_state
        kwargs = {
            f"key_{letter}": bool(getattr(kb, f"key_{letter}"))
            for letter in _LETTERS
        }
        return HexDcTeleopKeyboardState(**kwargs)

    @staticmethod
    def __jnt_state_to_dc(jnt) -> HexDcBaseJntState:
        return HexDcBaseJntState(
            position=np.asarray(jnt.position, dtype=np.float64),
            velocity=np.asarray(jnt.velocity, dtype=np.float64),
            effort=np.asarray(jnt.effort, dtype=np.float64),
        )

    @staticmethod
    def __pose_to_dc(pose: Pose) -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=HexDcBaseQuaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )

    @staticmethod
    def __manip_state_msg_to_dc(
            msg: HexRosRoboManipStateStamped) -> HexDcRoboManipStateStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_state.arm_state
        arm_state = HexDcRoboArmState(
            jnt=DataInterface.__jnt_state_to_dc(arm_msg.jnt),
            pose=DataInterface.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_state.grip_state
        grip_state = HexDcRoboGripState(
            jnt=DataInterface.__jnt_state_to_dc(grip_msg.jnt), )

        return HexDcRoboManipStateStamped(
            header=header,
            manip_state=HexDcRoboManipState(
                arm_state=arm_state,
                grip_state=grip_state,
            ),
        )
