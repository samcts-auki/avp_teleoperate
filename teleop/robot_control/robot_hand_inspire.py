# this file is legacy, need to fix.å
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_


import numpy as np
from enum import IntEnum
import time
import os
import sys
import threading
from multiprocessing import Process, shared_memory, Array, Lock

parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

inspire_tip_indices = [4, 9, 14, 19, 24]
Inspire_Num_Motors = 12
kTopicInspireCommand = "rt/inspire/cmd"
kTopicInspireState = "rt/inspire/state"
class InspireController:
    def __init__(self, left_hand_array, right_hand_array, 
                 inspire_hand_data_lock = None, inspire_hand_state_out = None, 
                 inspire_hand_action_out = None, fps = 200.0):
        print("Initialize InspireController")
        self.fps = fps
        self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)

        # initialize handcmd publisher and handstate subscriber
        self.InspireHandCmb_publisher = ChannelPublisher(kTopicInspireCommand, MotorCmds_)
        self.InspireHandCmb_publisher.Init()

        self.InspireHandState_subscriber = ChannelSubscriber(kTopicInspireState, MotorStates_)
        self.InspireHandState_subscriber.Init()

        self.inspire_hand_state_array = [0.0] * len(Inspire_JointIndex)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(state != 0.0 for state in self.inspire_hand_state_array):
                break
            time.sleep(0.01)
            print("[Inspire_Controller] Waiting to subscribe dds...")


        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.inspire_hand_state_array,
                                                                          inspire_hand_data_lock, inspire_hand_state_out, inspire_hand_action_out))
        hand_control_process.daemon = True
        hand_control_process.start()

        print("Initialize Inspire_Controller OK!\n")

    def _subscribe_state(self):
        while True:
            state_msg = self.InspireHandState_subscriber.Read()
            if state_msg is not None:
                for idx, id in enumerate(Inspire_JointIndex):
                    self.inspire_hand_state_array[idx] = state_msg.states[id].q
            time.sleep(0.002)

    def ctrl_inspire_hands(self, hand_q_target):
        """set current left, right hand motor state target q"""
        for idx, id in enumerate(Inspire_JointIndex):
            self.hand_msg.cmds[id].q = hand_q_target[idx]

        self.InspireHandCmb_publisher.Write(self.hand_msg)

    def control_process(self, left_hand_array, right_hand_array, inspire_hand_state_in, inspire_hand_data_lock = None, 
                                inspire_hand_state_out = None, inspire_hand_action_out = None):
            self.running = True

            dq = 0.0
            tau = 0.0
            kp = 5.00
            kd = 0.05
            # initialize hand cmd msg
            self.hand_msg  = MotorCmds_()
            self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_JointIndex))]
            for id in Inspire_JointIndex:
                self.hand_msg.cmds[id].dq  = dq
                self.hand_msg.cmds[id].tau = tau
                self.hand_msg.cmds[id].kp  = kp
                self.hand_msg.cmds[id].kd  = kd

            try:
                while self.running:
                    start_time = time.time()
                    # get dual hand skeletal point state from XR device
                    left_hand_mat  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                    right_hand_mat = np.array(right_hand_array[:]).reshape(25, 3).copy()
                    
                    left_q_target =  [0.0 for _ in range(6)]
                    right_q_target =  [0.0 for _ in range(6)]

                    if not np.all(left_hand_mat == 0.0): # if hand data has been initialized.
                        ref_left_value = left_hand_mat[inspire_tip_indices]
                        ref_right_value = right_hand_mat[inspire_tip_indices]

                        left_qpos  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
                        left_q_target = [1.7- left_qpos[i] for i in  [4, 6, 2, 0]]
                        left_q_target.append(1.2 - left_qpos[8])
                        left_q_target.append(0.5 - left_qpos[9])

                        right_qpos = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
                        right_q_target = [1.7 - right_qpos[i] for i in [4, 6, 2, 0]]
                        right_q_target.append(1.2 - right_qpos[8])
                        right_q_target.append(0.5 - right_qpos[9])
                        
                    # get dual hand action
                    action_data = np.concatenate((right_q_target, left_q_target))    
                    if inspire_hand_state_out and inspire_hand_action_out:
                        with inspire_hand_data_lock:
                            inspire_hand_state_out[:] = self.inspire_hand_state_array
                            inspire_hand_action_out[:] = action_data

                    self.ctrl_inspire_hands(action_data)
                    current_time = time.time()
                    time_elapsed = current_time - start_time
                    sleep_time = max(0, (1 / self.fps) - time_elapsed)
                    time.sleep(sleep_time)
            finally:
                print("Inspire_Controller has been closed.")

class Inspire_JointIndex(IntEnum):
    kLeftHandPinky = 0
    kLeftHandRing = 1
    kLeftHandMiddle = 2
    kLeftHandIndex = 3
    kLeftHandThumbBend = 4
    kLeftHandThumbRot = 5
    kRightHandPinky = 6
    kRightHandRing = 7
    kRightHandMiddle = 8
    kRightHandIndex = 9
    kRightHandThumbBend = 10
    kRightHandThumbRot = 11