#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from doctest import master
import os
import sys
import time
import traceback
import threading
from typing import Optional

import numpy as np
from hex_util_ros import part2se3, se32part

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from sympy import false
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

from TrajectoryController import Move2TargetPlanner

ARM_DOF = 6
GRIP_DOF = 1

EXTRA_MASS = 0.1

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
        
        self.__extra_force = -self.__dyn_util.get_gravity(
        ) * EXTRA_MASS
        
        ### control presets
        self.__arm_start_pos = np.asarray(
            self.__force_feedback_param["arm_start_pos"], dtype=np.float64)
        self.__arm_end_pos = np.asarray(self.__force_feedback_param["arm_end_pos"],
                                        dtype=np.float64)
        self.__arm_start_pose = self.__dyn_util.forward_kinematics(
            self.__arm_start_pos)[-1]
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
        self.__arm_master_kp = np.asarray(
            self.__force_feedback_param["arm_master_kp"], dtype=np.float64)
        self.__arm_master_kd = np.asarray(
            self.__force_feedback_param["arm_master_kd"], dtype=np.float64)
        self.__grip_master_kp = np.asarray(
            self.__force_feedback_param["grip_master_kp"], dtype=np.float64)
        self.__grip_master_kd = np.asarray(
            self.__force_feedback_param["grip_master_kd"], dtype=np.float64)

        self.__arm_slave_kp = np.asarray(
            self.__force_feedback_param["arm_slave_kp"], dtype=np.float64)
        self.__arm_slave_kd = np.asarray(
            self.__force_feedback_param["arm_slave_kd"], dtype=np.float64)
        self.__grip_slave_kp = np.asarray(
            self.__force_feedback_param["grip_slave_kp"], dtype=np.float64)
        self.__grip_slave_kd = np.asarray(
            self.__force_feedback_param["grip_slave_kd"], dtype=np.float64)
        self.__arm_master_deadzone = np.asarray(
            self.__force_feedback_param["arm_master_deadzone"], dtype=np.float64)
        self.__arm_master_clip = np.asarray(
            self.__force_feedback_param["arm_master_clip"], dtype=np.float64)
        self.__arm_slave_deadzone = np.asarray(
            self.__force_feedback_param["arm_slave_deadzone"], dtype=np.float64)
        self.__arm_slave_clip = np.asarray(
            self.__force_feedback_param["arm_slave_clip"], dtype=np.float64)

        ### threads
        self.__stop_event = threading.Event()
        self.__start_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)
        
        self.__motor_cnt = 6
        
        self._logd = self.__data_interface.logd

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

    def __build_stable_ctrl(self, jnt_pos: np.ndarray) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=jnt_pos,
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_stable_kp.copy(),
                kd=self.__arm_stable_kd.copy(),
                lim_vel=2.0 * np.ones(ARM_DOF,dtype=np.float64),
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

    def __build_follow_ctrl(
            self,
            arm_jnt_pos: Optional[np.ndarray] = None,
            arm_jnt_eff: Optional[np.ndarray] = None,
            arm_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_pos: Optional[np.ndarray] = None,
            grip_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_eff: Optional[np.ndarray] = None) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=arm_jnt_pos if arm_jnt_pos is not None else self.__arm_start_pos.copy(),
                vel=arm_jnt_vel if arm_jnt_vel is not None else np.zeros(ARM_DOF),
                eff=arm_jnt_eff if arm_jnt_eff is not None else np.zeros(ARM_DOF),
                kp=self.__arm_slave_kp.copy(),
                kd=self.__arm_slave_kd.copy(),
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
                vel=grip_jnt_vel if grip_jnt_vel is not None else np.zeros(GRIP_DOF),
                eff=grip_jnt_eff if grip_jnt_eff is not None else np.zeros(GRIP_DOF),
                kp=self.__grip_slave_kp.copy(),
                kd=self.__grip_slave_kd.copy(),
                lim_vel=np.zeros(GRIP_DOF),
                lim_acc=np.zeros(GRIP_DOF),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    def __build_feedback_ctrl(self,
            arm_jnt_pos: Optional[np.ndarray] = None,
            arm_jnt_eff: Optional[np.ndarray] = None,
            arm_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_pos: Optional[np.ndarray] = None,
            grip_jnt_vel: Optional[np.ndarray] = None,
            grip_jnt_eff: Optional[np.ndarray] = None) -> HexDcRoboManipCtrl:
        # MIT mode with master-side PD gains: the driver/sim adds the model
        # gravity + coriolis compensation (via `grav`), so the commanded
        # effort combines PD feedback with the compensation torque.
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=np.asarray(arm_jnt_pos, dtype=np.float64) if arm_jnt_pos is not None else np.zeros(ARM_DOF),
                vel=np.asarray(arm_jnt_vel, dtype=np.float64)  if arm_jnt_vel is not None else np.zeros(ARM_DOF),
                eff=np.asarray(arm_jnt_eff, dtype=np.float64)  if arm_jnt_eff is not None else np.zeros(ARM_DOF),
                kp=self.__arm_master_kp.copy(),
                kd=self.__arm_master_kd.copy(),
                lim_vel=np.zeros(ARM_DOF),
                lim_acc=np.zeros(ARM_DOF),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=grip_jnt_pos if grip_jnt_pos is not None else np.zeros(GRIP_DOF),
                vel=grip_jnt_vel if grip_jnt_vel is not None else np.zeros(GRIP_DOF),
                eff=grip_jnt_eff if grip_jnt_eff is not None else np.zeros(GRIP_DOF),
                kp=self.__grip_master_kp.copy(),
                kd=self.__grip_master_kd.copy(),
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
        prev_s = False
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

            curr_s = bool(keys.key_s)
            if curr_s and not prev_s:
                self.__start_event.set()
            prev_s = curr_s
            

    def __move_to_stable(self, phase: str, is_start: bool = True):
        self.__data_interface.logi(
            f"[arm_force_feedback]: move to {phase} position")
        
        stable_pos = self.__arm_start_pos if is_start else self.__arm_end_pos
        duration = 10

        
        master_state = self.__data_interface.get_master_manip_state(
            latest=True)
        while master_state is None and self.__data_interface.ok():
            self.__data_interface.logw(
                "waiting for master state...")
            time.sleep(0.1)
            master_state = self.__data_interface.get_master_manip_state(
                latest=True)
        
        slave_state = self.__data_interface.get_slave_manip_state(
            latest=True)
        while slave_state is None and self.__data_interface.ok():
            self.__data_interface.logw(
                "waiting for slave state...")
            time.sleep(0.1)
            slave_state = self.__data_interface.get_slave_manip_state(
                latest=True)

        # initialize planners
        if master_state is not None and slave_state is not None:
            master_jnt = np.asarray(
                master_state.manip_state.arm_state.jnt.position,
                dtype=np.float64)
            slave_jnt = np.asarray(
                slave_state.manip_state.arm_state.jnt.position,
                dtype=np.float64)

            master_planner = Move2TargetPlanner(
                        master_jnt, stable_pos, duration)
            slave_planner = Move2TargetPlanner(
                        slave_jnt, stable_pos, duration)
            
            master_planner.start_trajectory()   
            slave_planner.start_trajectory()


        while self.__data_interface.ok():

            master_done = slave_done = False

            if master_planner is not None:
                master_target, master_done = master_planner.get_target_position()
                self.__data_interface.pub_master_manip_ctrl(
                    self.__build_stable_ctrl(jnt_pos=master_target))

            if slave_planner is not None:
                slave_target, slave_done = slave_planner.get_target_position()
                self.__data_interface.pub_slave_manip_ctrl(
                    self.__build_stable_ctrl(jnt_pos=slave_target))

            if master_planner is not None and slave_planner is not None \
                    and master_done and slave_done:
                break

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
        
        self.__data_interface.logi("press 's' to start force feedback control")
        self.__data_interface.logi("press 'q' to exit force feedback control")
        
        while self.__is_running() and not self.__start_event.is_set():
            time.sleep(0.1)
        
        self.__data_interface.logi("start force feedback control")
        
        self.__feedback()
            
    def __feedback(self):
        
        master_pos = None       
        master_vel = None
        
        slave_pos = None
        slave_vel = None
        
        master_target_pos = slave_target_pos = None

        while self.__is_running():
            master_state = self.__data_interface.get_master_manip_state(latest=True)
            slave_state = self.__data_interface.get_slave_manip_state(latest=True)

            ## master
            if master_state is not None:

                # state
                master_pos = np.asarray(master_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                master_vel = np.asarray(master_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)


            ## slave
            if slave_state is not None:

                slave_pos = np.asarray(slave_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                slave_vel = np.asarray(slave_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)

            if master_pos is not None and slave_pos is not None:

                try:

                    master_target_pos = self.__compute_effective_target(master_pos, slave_pos,
                        self.__arm_master_deadzone, self.__arm_master_clip)
                    
                    slave_target_pos = self.__compute_effective_target(slave_pos, master_pos,
                        self.__arm_slave_deadzone, self.__arm_slave_clip)

                except Exception:
                    traceback.print_exc()
                
                
                self._logd(f"master_target_pos{master_target_pos}, slave_target_pos{slave_target_pos}") 
                
                self.__data_interface.pub_slave_manip_ctrl(
                    self.__build_follow_ctrl(arm_jnt_pos=slave_target_pos, arm_jnt_vel=master_vel))

                self.__data_interface.pub_master_manip_ctrl(
                    self.__build_feedback_ctrl(arm_jnt_pos=master_target_pos, arm_jnt_vel=slave_vel))
                
            self.__data_interface.sleep()
            

    def __compute_effective_target(
            self, 
            current: np.ndarray, 
            target: np.ndarray, 
            deadzone:  Optional[np.ndarray], 
            clip_bound: Optional[np.ndarray] = None
        ) -> np.ndarray:
        """
        Apply deadzone compensation and saturation clipping.
    
        Args:
            current: Current joint positions, shape (N,)
            target: Desired joint positions, shape (N,)
            deadzone: Per-joint deadzone width, shape (N,)
            clip_bound: Per-joint saturation limit, shape (N,). None = no clipping.
        
        Returns:
            Effective target after deadzone + clipping, shape (N,)
        """
        e = target - current
        
        if deadzone is not None:
            e_abs = np.fabs(e)
            zero_mask = (e_abs <= deadzone)
            e[zero_mask] = 0.0
            e[~zero_mask] = e[~zero_mask] - np.sign(e[~zero_mask]) * deadzone[~zero_mask]
        
        if clip_bound is not None:
            e = np.clip(e, -clip_bound, clip_bound)
        return current + e     


def main():
    arm_force_feedback = ArmForceFeedback()
    try:
        arm_force_feedback.start()
        arm_force_feedback.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
