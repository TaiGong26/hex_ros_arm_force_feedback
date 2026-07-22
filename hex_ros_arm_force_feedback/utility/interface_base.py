#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_robo import HexDcRoboManipCtrl
from hex_util_msg.dataclass.dataclass_robo import HexDcRoboManipStateStamped
from hex_util_msg.dataclass.dataclass_teleop import HexDcTeleopKeyboardState


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}
        self._model_param = {}
        self._impedance_param = {}

        ### rx msg queues
        self._manip_state_deque = deque(maxlen=100)
        self._keyboard_deque = deque(maxlen=100)

        ### name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_model_param(self) -> dict:
        return self._model_param

    def get_impedance_param(self) -> dict:
        return self._impedance_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_manip_ctrl(self, out: HexDcRoboManipCtrl):
        raise NotImplementedError("InterfaceBase.pub_manip_ctrl")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # manip state
    def get_manip_state(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboManipStateStamped]:
        return self.deque_helper(self._manip_state_deque, latest)

    # keyboard state
    def get_keyboard_state(
        self,
        latest: bool = False,
    ) -> Optional[HexDcTeleopKeyboardState]:
        return self.deque_helper(self._keyboard_deque, latest)
