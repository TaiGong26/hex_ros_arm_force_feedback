#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import time

import numpy as np
import rospy

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
        rospy.init_node(name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 500.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### parameters
        self._rate_param.update({
            "teleop": rospy.get_param('~rate_teleop', 100.0),
        })
        self._model_param = {
            "urdf":
            rospy.get_param('~model_urdf', ""),
            "frame_id":
            rospy.get_param('~model_frame_id', "base_link"),
            "pose_end_in_flange":
            list(
                rospy.get_param('~pose_end_in_flange',
                                [0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])),
        }
        self._force_feedback_param = {
            "gravity":
            list(rospy.get_param('~gravity', [0.0, 0.0, -9.81])),
            "arm_start_pos":
            list(
                rospy.get_param('~arm_start_pos',
                                [0.0, -1.5, 3.0, 0.07, 0.0, 0.0])),
            "arm_end_pos":
            list(
                rospy.get_param('~arm_end_pos',
                                [0.0, -1.5, 3.0, 0.07, 0.0, 0.0])),
            "grip_stable_pos":
            list(rospy.get_param('~grip_stable_pos', [0.5])),
            "arm_stable_kp":
            list(
                rospy.get_param('~arm_stable_kp',
                                [200.0, 200.0, 250.0, 150.0, 100.0, 100.0])),
            "arm_stable_kd":
            list(rospy.get_param('~arm_stable_kd', [5.0, 5.0, 5.0, 5.0, 2.0, 2.0])),
            "grip_stable_kp":
            list(rospy.get_param('~grip_stable_kp', [10.0])),
            "grip_stable_kd":
            list(rospy.get_param('~grip_stable_kd', [0.5])),
            "arm_pos_threshold":
            rospy.get_param('~arm_pos_threshold', 0.1),
            "arm_master_kp":
            list(
                rospy.get_param('~arm_master_kp',
                                [0.0, 0.0, 0.0, 150.0, 100.0, 100.0])),
            "arm_master_kd":
            list(
                rospy.get_param('~arm_master_kd',
                                [0.0, 0.0, 0.0, 5.0, 2.0, 2.0])),
            "grip_master_kp":
            list(rospy.get_param('~grip_master_kp', [10.0])),
            "grip_master_kd":
            list(rospy.get_param('~grip_master_kd', [0.5])),
            "arm_slave_kp":
            list(
                rospy.get_param('~arm_slave_kp',
                                [0.0, 0.0, 0.0, 150.0, 100.0, 100.0])),
            "arm_slave_kd":
            list(
                rospy.get_param('~arm_slave_kd',
                                [0.0, 0.0, 0.0, 5.0, 2.0, 2.0])),
            "grip_slave_kp":
            list(rospy.get_param('~grip_slave_kp', [10.0])),
            "grip_slave_kd":
            list(rospy.get_param('~grip_slave_kd', [0.5])),
            "arrive_threshold":
            rospy.get_param('~arrive_threshold', 0.06),
        }

        ### publisher

        self.__master_manip_ctrl_pub = rospy.Publisher(
            'master/manip_ctrl',
            HexRosRoboManipCtrlStamped,
            queue_size=10,
        )
        self.__slave_manip_ctrl_pub = rospy.Publisher(
            'slave/manip_ctrl',
            HexRosRoboManipCtrlStamped,
            queue_size=10,
        )

        ### subscriber
        self.__manip_state_sub = rospy.Subscriber(
            'manip_state',
            HexRosRoboManipStateStamped,
            self.__manip_state_callback,
        )
        self.__keyboard_sub = rospy.Subscriber(
            'teleop_keyboard_state',
            HexRosTeleopKeyboardStateStamped,
            self.__keyboard_callback,
        )
        self.__manip_state_sub
        self.__keyboard_sub

        ### master/slave subscriber
        self.__master_manip_state_sub = rospy.Subscriber(
            'master/manip_state',
            HexRosRoboManipStateStamped,
            self.__master_manip_state_callback,
        )
        self.__slave_manip_state_sub = rospy.Subscriber(
            'slave/manip_state',
            HexRosRoboManipStateStamped,
            self.__slave_manip_state_callback,
        )
        self.__master_manip_state_sub
        self.__slave_manip_state_sub

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################

    def pub_master_manip_ctrl(self, out: HexDcRoboManipCtrl):
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = rospy.Time.now()
        msg.manip_ctrl = HexRosRoboManipCtrl(
            arm_ctrl=self.__arm_ctrl_to_msg(out.arm_ctrl),
            grip_ctrl=self.__grip_ctrl_to_msg(out.grip_ctrl),
        )
        self.__master_manip_ctrl_pub.publish(msg)

    def pub_slave_manip_ctrl(self, out: HexDcRoboManipCtrl):
        msg = HexRosRoboManipCtrlStamped()
        msg.header.stamp = rospy.Time.now()
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
    def __pose_to_dc(pose) -> HexDcBasePose:
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

    def __manip_state_msg_to_dc(
            self,
            msg: HexRosRoboManipStateStamped) -> HexDcRoboManipStateStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.secs),
                nsecs=int(msg.header.stamp.nsecs),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_state.arm_state
        arm_state = HexDcRoboArmState(
            jnt=self.__jnt_state_to_dc(arm_msg.jnt),
            pose=self.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_state.grip_state
        grip_state = HexDcRoboGripState(jnt=self.__jnt_state_to_dc(
            grip_msg.jnt), )

        return HexDcRoboManipStateStamped(
            header=header,
            manip_state=HexDcRoboManipState(
                arm_state=arm_state,
                grip_state=grip_state,
            ),
        )
