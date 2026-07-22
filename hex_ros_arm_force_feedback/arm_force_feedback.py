#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import os
import sys
import time
import traceback
import threading

import numpy as np
from hex_util_ros import part2se3, se32part

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
)
from hex_util_ros import HexDynUtilY6

ARM_DOF = 6
GRIP_DOF = 1


class ArmForceFeedback:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("arm_force_feedback")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__force_feedback_param = self.__data_interface.get_force_feedback_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")
        self.__data_interface.logi(f"model urdf: {self.__model_param['urdf']}")

        ### dynamics
        self.__gravity = np.asarray(self.__force_feedback_param["gravity"],
                                    dtype=np.float64)
        self.__dyn_util = HexDynUtilY6(
            model_path=self.__model_param["urdf"],
            last_link="link_6",
            pose_end_in_flange=np.asarray(
                self.__model_param["pose_end_in_flange"], dtype=np.float64),
            gravity=self.__gravity,
        )

        ### control presets
        self.__arm_start_pos = np.asarray(
            self.__force_feedback_param["arm_start_pos"], dtype=np.float64)
        self.__arm_end_pos = np.asarray(self.__force_feedback_param["arm_end_pos"],
                                        dtype=np.float64)
        self.__arm_start_pose = self.__dyn_util.forward_kinematics(
            self.__arm_start_pos)[-1]
        self.__arm_pos_threshold = self.__force_feedback_param["arm_pos_threshold"]
        self.__grip_stable_pos = np.asarray(
            self.__force_feedback_param["grip_stable_pos"], dtype=np.float64)
        self.__arm_stable_kp = np.asarray(self.__force_feedback_param["arm_stable_kp"],
                                   dtype=np.float64)
        self.__arm_stable_kd = np.asarray(self.__force_feedback_param["arm_stable_kd"],
                                   dtype=np.float64)
        self.__grip_stable_kp = np.asarray(self.__force_feedback_param["grip_stable_kp"],
                                    dtype=np.float64)
        self.__grip_stable_kd = np.asarray(self.__force_feedback_param["grip_stable_kd"],
                                    dtype=np.float64)
        self.__arm_impedance_kp = np.asarray(
            self.__force_feedback_param["arm_impedance_kp"], dtype=np.float64)
        self.__arm_impedance_kd = np.asarray(
            self.__force_feedback_param["arm_impedance_kd"], dtype=np.float64)
        self.__grip_impedance_kp = np.asarray(
            self.__force_feedback_param["grip_impedance_kp"], dtype=np.float64)
        self.__grip_impedance_kd = np.asarray(
            self.__force_feedback_param["grip_impedance_kd"], dtype=np.float64)
        self.__arrive_threshold = self.__force_feedback_param["arrive_threshold"]

        ### threads
        self.__stop_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)

    def __is_running(self):
        return self.__data_interface.ok() and not self.__stop_event.is_set()

    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self):
        self.__stop_event.clear()
        self.__teleop_thread.start()
        self.__init_process()

    def run(self):
        try:
            self.__work_process()
        except KeyboardInterrupt:
            pass
        except Exception:
            traceback.print_exc()
        finally:
            self.stop()

    def stop(self):
        self.__stop_event.set()
        if self.__teleop_thread.is_alive():
            self.__teleop_thread.join()
        self.__exit_process()
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass

    ##############################################################
    # Control builders
    ##############################################################
    @staticmethod
    def __default_pose() -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            orientation=HexDcBaseQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

    def __build_stable_ctrl(self, is_start: bool = True) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=self.__arm_start_pos.copy()
                if is_start else self.__arm_end_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_stable_kp.copy(),
                kd=self.__arm_stable_kd.copy(),
                lim_vel=1.0 * np.ones(ARM_DOF,dtype=np.float64),
                lim_acc=100 * np.ones(ARM_DOF,dtype=np.float64),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=self.__grip_stable_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.ones(GRIP_DOF),
                kp=self.__grip_stable_kp.copy(),
                kd=self.__grip_stable_kd.copy(),
                lim_vel=np.array([0.5]),
                lim_acc=np.array([1.0]),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    def __build_impedance_ctrl(
            self,
            arm_jnt_pos: np.ndarray = None,
            grip_jnt_pos: np.ndarray = None) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=arm_jnt_pos
                if arm_jnt_pos is not None else self.__arm_start_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_impedance_kp.copy(),
                kd=self.__arm_impedance_kd.copy(),
                lim_vel=np.zeros(ARM_DOF),
                lim_acc=np.zeros(ARM_DOF),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=grip_jnt_pos
                if grip_jnt_pos is not None else self.__grip_stable_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.zeros(GRIP_DOF),
                kp=self.__grip_impedance_kp.copy(),
                kd=self.__grip_impedance_kd.copy(),
                lim_vel=np.zeros(GRIP_DOF),
                lim_acc=np.zeros(GRIP_DOF),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    ##############################################################
    # Processes
    ##############################################################
    def __teleop_process(self):
        prev_q = False
        while self.__is_running():
            time.sleep(self.__teleop_dt)

            keys = self.__data_interface.get_keyboard_state(latest=True)
            if keys is None:
                continue

            curr_q = bool(keys.key_q)
            if curr_q and not prev_q:
                self.__data_interface.logi("[arm_force_feedback]: stop and exit")
                self.__stop_event.set()
            prev_q = curr_q

    def __arrived_at(self, jnt_pos: np.ndarray,
                     target: np.ndarray) -> bool:
        if jnt_pos.shape != target.shape:
            return False
        err = target - jnt_pos
        return bool(np.fabs(err).max() < self.__arrive_threshold)

    def __move_to_stable(self, phase: str, is_start: bool = True):
        self.__data_interface.logi(
            f"[arm_force_feedback]: move to {phase} position")
        stable_ctrl = self.__build_stable_ctrl(is_start)
        stable_pos = self.__arm_start_pos if is_start else self.__arm_end_pos
        while self.__data_interface.ok():
            master_arrived = False
            slave_arrived = False

            # master state
            master_state = self.__data_interface.get_master_manip_state(
                latest=True)
            if master_state is not None:
                master_jnt_pos = np.asarray(
                    master_state.manip_state.arm_state.jnt.position,
                    dtype=np.float64,
                )
                master_arrived = self.__arrived_at(master_jnt_pos, stable_pos)

            # slave state
            slave_state = self.__data_interface.get_slave_manip_state(
                latest=True)
            if slave_state is not None:
                slave_jnt_pos = np.asarray(
                    slave_state.manip_state.arm_state.jnt.position,
                    dtype=np.float64,
                )
                slave_arrived = self.__arrived_at(slave_jnt_pos, stable_pos)

            if master_arrived and slave_arrived:
                break

            self.__data_interface.pub_master_manip_ctrl(stable_ctrl)
            self.__data_interface.pub_slave_manip_ctrl(stable_ctrl)
            self.__data_interface.sleep()

    def __init_process(self):
        try:
            self.__move_to_stable("init", is_start=True)
        except Exception:
            traceback.print_exc()

    def __exit_process(self):
        try:
            self.__move_to_stable("exit", is_start=False)
        except Exception:
            traceback.print_exc()

    def __work_process(self):
        self.__data_interface.logi("[arm_force_feedback]: start impedance control")
        # while self.__is_running():
        #     state = self.__data_interface.get_manip_state(latest=True)
        #     if state is not None:
        #         pos = np.array([
        #             state.manip_state.arm_state.pose.position.x,
        #             state.manip_state.arm_state.pose.position.y,
        #             state.manip_state.arm_state.pose.position.z
        #         ])

        #         pos_err = self.__arm_start_pose[0] - pos
        #         max_err = np.max(np.abs(pos_err))
        #         ratio = 1.0 if max_err < self.__arm_pos_threshold else self.__arm_pos_threshold / max_err
        #         pos_err = pos_err * ratio
        #         tar_pos = pos + pos_err

        #         ik_success, tar_jnt_pos = self.__dyn_util.inverse_kinematics_analytic(
        #             (tar_pos, self.__arm_start_pose[1]),
        #             state.manip_state.arm_state.jnt.position)
        #         if not ik_success:
        #             tar_jnt_pos = state.manip_state.arm_state.jnt.position
        #             print(f"[arm_force_feedback]: inverse kinematics failed")
                    
        #         # self.__data_interface.pub_manip_ctrl(
        #         #     self.__build_impedance_ctrl(tar_jnt_pos))
        #     self.__data_interface.sleep()


def main():
    arm_force_feedback = ArmForceFeedback()
    try:
        arm_force_feedback.start()
        arm_force_feedback.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
