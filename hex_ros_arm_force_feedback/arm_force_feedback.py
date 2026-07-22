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
        
        
        self.__arm_slave_follow_kp = np.asarray(
            self.__force_feedback_param["arm_slave_follow_kp"], dtype=np.float64)
        self.__arm_slave_follow_kd = np.asarray(
            self.__force_feedback_param["arm_slave_follow_kd"], dtype=np.float64)
        self.__grip_slave_follow_kp = np.asarray(
            self.__force_feedback_param["grip_slave_follow_kp"], dtype=np.float64)
        self.__grip_slave_follow_kd = np.asarray(
            self.__force_feedback_param["grip_slave_follow_kd"], dtype=np.float64)
        self.__arrive_threshold = self.__force_feedback_param["arrive_threshold"]

        ### threads
        self.__stop_event = threading.Event()
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
                lim_vel=10.0 * np.ones(ARM_DOF,dtype=np.float64),
                lim_acc=10 * np.ones(ARM_DOF,dtype=np.float64),
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
            grip_jnt_pos: Optional[np.ndarray] = None) -> HexDcRoboManipCtrl:
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
                eff= arm_jnt_eff if arm_jnt_eff is not None else np.zeros(ARM_DOF),
                kp=self.__arm_slave_follow_kp.copy(),
                kd=self.__arm_slave_follow_kd.copy(),
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

    def __build_feedback_ctrl(self, 
            arm_jnt_pos: Optional[np.ndarray] = None,
            arm_jnt_eff: Optional[np.ndarray] = None,) -> HexDcRoboManipCtrl:
        # MIT mode with zero gains: the driver/sim adds the model gravity +
        # coriolis compensation (via `grav`), so the only commanded effort is
        # the torque that holds the extra end-effector payload.
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.MIT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=  np.asarray(arm_jnt_pos, dtype=np.float64) if arm_jnt_pos is not None else np.zeros(ARM_DOF) ,
                vel=np.zeros(ARM_DOF),
                eff=np.asarray(arm_jnt_eff, dtype=np.float64),
                kp=np.zeros(ARM_DOF),
                kd=np.zeros(ARM_DOF),
                lim_vel=np.zeros(ARM_DOF),
                lim_acc=np.zeros(ARM_DOF),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=np.zeros(GRIP_DOF),
                vel=np.zeros(GRIP_DOF),
                eff=np.zeros(GRIP_DOF),
                kp=np.zeros(GRIP_DOF),
                kd=np.zeros(GRIP_DOF),
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
            
        # self.__follow_test()
        self.__feedbcak_test()


    def __follow_test(self):
        
        while self.__is_running():
        
            master_state = self.__data_interface.get_master_manip_state(latest=True)
            slave_state = self.__data_interface.get_slave_manip_state(latest=True)

            master_q = None
            
            extra_tau = np.zeros(ARM_DOF)
            
            ### master
            if master_state is not None:
                master_q = np.asarray(master_state.manip_state.arm_state.jnt.position,
                                dtype=np.float64)
            
            # self._logd(f"slave state {slave_state} master_q{master_q } ,slave_q{slave_q}")
            
            if extra_tau is not None:
                self.__data_interface.pub_master_manip_ctrl(
                        self.__build_feedback_ctrl(arm_jnt_eff=extra_tau))
            
            if master_q is not None:
                
                self.__data_interface.pub_slave_manip_ctrl(
                    self.__build_follow_ctrl(master_q))
                
            self.__data_interface.sleep()
            
    
    def __feedbcak_test(self):
        
        res_feedback=False
        master_pos = None       
        master_vel = None
        
        slave_pos = None
        slave_vel = None
        slave_eff = None

        comp_weight = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        comp_deadzone = np.array([7.0, 7.0, 7.0, 3.0, 2.0, 2.0])
        
        while self.__is_running():
            master_state = self.__data_interface.get_master_manip_state(latest=True)
            slave_state = self.__data_interface.get_slave_manip_state(latest=True)

            master_cmd_eff = master_cmd_pos = None

            ## master
            if master_state is not None:
                
                # state
                master_pos = np.asarray(master_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                master_vel = np.asarray(master_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)
                
                _, c_mat, g_vec, _, _ = self.__dyn_util.dynamic_params(master_pos,master_vel)
                
                master_tau_comp =  c_mat @ master_vel + g_vec
                
                if res_feedback:
                    master_tau_comp -= self.__deadzone(
                        slave_res_eff,
                        comp_deadzone,
                    ) * comp_weight
            
                self.__data_interface.pub_master_manip_ctrl(
                    self.__build_feedback_ctrl(master_pos,master_tau_comp))
            
            ## slave

            if slave_state is not None:
                slave_pos = np.asarray(slave_state.manip_state.arm_state.jnt.position, dtype=np.float64)
                slave_vel = np.asarray(slave_state.manip_state.arm_state.jnt.velocity, dtype=np.float64)
                slave_eff = np.asarray(slave_state.manip_state.arm_state.jnt.effort, dtype=np.float64)

                _, c_mat, g_vec, _, _ = self.__dyn_util.dynamic_params(slave_pos, slave_vel)
                
                slave_tau_comp = c_mat @ slave_vel + g_vec

                slave_res_eff = slave_eff.copy()
                    
                slave_res_eff -= slave_tau_comp
                    
                # slave_res_eff -= slave_tau_comp

                if master_pos is not None:
                    slave_res_q = slave_pos - master_pos
                    if np.fabs(slave_res_q).max() < 0.5 and not res_feedback:
                        res_feedback = True
                    
                    # cmd_pos , cmd_eff = cmds
                    self.__data_interface.pub_slave_manip_ctrl(
                        self.__build_follow_ctrl(master_pos,slave_tau_comp))
                    
                    
                    # hexarm_slave_client.set_cmds(cmds)
            
    ##############################################################
    # tools
    ##############################################################
    def __deadzone(self ,var, deadzone):
        res = var.copy()
        zero_mask = np.fabs(res) < deadzone
        res[zero_mask] = 0.0
        res[~zero_mask] -= np.sign(res[~zero_mask]) * deadzone[~zero_mask]
        return res
            
    
def main():
    arm_force_feedback = ArmForceFeedback()
    try:
        arm_force_feedback.start()
        arm_force_feedback.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
