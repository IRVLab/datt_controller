#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 25 12:04:34 2022

@author: enan
"""

import rospy
import time
from aquacore.msg import AutopilotModes
from rrcomm_autopilot_client import AutopilotClient
from datt_controller.msg import Datt


class Counter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0 
        self.avg = 0
        self.sum = 0
        self.count = 0
    
    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n 
        self.avg = self.sum / self.count



class RobotController:
    def __init__(self):
        rospy.init_node('datt_controller_aqua')

        self.params = {}
        # params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH
        self.params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
        self.ac = AutopilotClient(self.params)

        # self.rate = rospy.Rate(1)
        self.attentive_meter = Counter()
        self.right_not_attentive_meter = Counter()
        self.left_not_attentive_meter = Counter()
        self.pred_sub = rospy.Subscriber('/detection/prediction', Datt, self.Callback, queue_size=1)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Rospy shutting down.")
    
    
    def Callback(self, pred_topic):
        if pred_topic.prediction == 'Attentive':
            self.attentive_meter.update(pred_topic.mask)                

        else: 
            if pred_topic.mask > 0:
                self.right_not_attentive_meter.update(pred_topic.mask)
            else:
                self.left_not_attentive_meter.update(pred_topic.mask)

        if self.attentive_meter.count > 3:
            print('The subject is ATTENTIVE.')
            self.agreement()
            # time.sleep(10)

            self.attentive_meter.reset()
            self.right_not_attentive_meter.reset()
            self.left_not_attentive_meter.reset()

            rospy.signal_shutdown("INITIATE Interaction.")

        elif self.right_not_attentive_meter.count > 3:
            print('The subject is NOT ATTENTIVE (looking towards RIGHT)')
            print('Drawing Attention.')
            self.seeking_attention()
            time.sleep(2)

            print('Turning:', pred_topic.rc)
            self.right_manuever(pred_topic)
            # time.sleep(1)

            self.attentive_meter.reset()
            self.right_not_attentive_meter.reset()
            self.left_not_attentive_meter.reset()

            rospy.signal_shutdown("INITIATE Interaction.")

        elif self.left_not_attentive_meter.count > 3:
            print('The subject is NOT ATTENTIVE (looking towards LEFT)')
            print('Drawing Attention.')
            self.seeking_attention()
            time.sleep(2)

            print('Turning:', pred_topic.rc)
            self.left_manuever(pred_topic)
            time.sleep(1)

            self.attentive_meter.reset()
            self.right_not_attentive_meter.reset()
            self.left_not_attentive_meter.reset()

            rospy.signal_shutdown("INITIATE Interaction.")

        # self.rate.sleep()

    
    def agreement(self):
        d = self.ac.current_depth
        vx = 0.1
        vz = 0

        rospy.loginfo(' Pitching down...')
        self.ac.do_relative_angle_change([0, 45, 0], d, vx, vz, dt_in_sec=3) # [RPY]: P +ve down
        rospy.loginfo(' Pitching up...')
        self.ac.do_relative_angle_change([0, -60, 0], d, vx, vz, dt_in_sec=3)        
        self.ac.do_straight_line_rel(1, d, 0.1, 0)
        rospy.loginfo(' Done Acknowledging ATTENTION!!!')

    def seeking_attention(self):
        d = self.ac.current_depth
        vx = 0.1
        vz = 0

        rospy.loginfo(' Rolling right...')
        self.ac.do_relative_angle_change([20, 0, 0], d, vx, vz, dt_in_sec=1.5) # [RPY]: P +ve down
        rospy.loginfo(' Rolling left...')
        self.ac.do_relative_angle_change([-20, 0, 0], d, vx, vz, dt_in_sec=1.5)        
        self.ac.do_straight_line_rel(1, d, 0.1, 0)
        rospy.loginfo(' Done Trying to GRAB Attention!!!')

    def right_manuever(self, pred_topic):
        d = self.ac.current_depth
        vx = 0.25
        vz = 0

        pseudo_dist = abs(pred_topic.mask)//3
        print(' Pseudo Distance: ', pseudo_dist)

        rospy.loginfo(' Start RIGHT maneuver: Yaw right...')
        self.ac.do_relative_angle_change([0, 15, -30], d, vx, vz, dt_in_sec=2) # [RPY]: P +ve down, Y -ve right
        rospy.loginfo(' Going right line...')   
        time.sleep(2)
        self.ac.do_straight_line_rel(pseudo_dist, d, vx, 0)
        rospy.loginfo(' Yaw left...')
        time.sleep(2)
        self.ac.do_relative_angle_change([0, 15, 60], d, vx, vz, dt_in_sec=2.5) # [RPY]: P +ve down
        time.sleep(1)
        self.ac.do_straight_line_rel(1, d, 0.1, 0)
        rospy.loginfo(' Done right maneuver!!!')

    def left_manuever(self, pred_topic):
        d = self.ac.current_depth
        vx = 0.25
        vz = 0

        pseudo_dist = abs(pred_topic.mask)//3
        print(' Pseudo Distance: ', pseudo_dist)

        rospy.loginfo(' Start LEFT maneuver: Yaw left...')
        self.ac.do_relative_angle_change([0, 5, 30], d, vx, vz, dt_in_sec=2) # [RPY]: P +ve down
        rospy.loginfo(' Going right line...')   
        time.sleep(1)
        self.ac.do_straight_line_rel(pseudo_dist, d, vx, 0)
        rospy.loginfo(' Yaw left...')
        time.sleep(2)
        self.ac.do_relative_angle_change([0, 5, -80], d, vx, vz, dt_in_sec=3.5) # [RPY]: P +ve down
        time.sleep(1)
        self.ac.do_straight_line_rel(1, d, 0.1, 0)
        rospy.loginfo(' Done left maneuver!!!')




RobotController()
