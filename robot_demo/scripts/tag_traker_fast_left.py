#!/usr/bin/env python
# -*- coding: utf-8 -*-
import CMDcontrol
import rospy
import tf
import time
import threading
import numpy as np
from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from kickBallOnly import kick_ball



def angle_between(v1, v2):
    cosTh = np.dot(v1, v2)
    sinTh = np.cross(v1, v2)
    angle = np.rad2deg(np.arctan2(sinTh, cosTh))[2]
    return angle

class TagConverter():
    def __init__(self):
        self.p_0 = np.array([-0.1429473853758271, -0.004343588010841607, 0.1392410096834090876])
        self.p_1 = np.array([-0.043167954694022376, -0.00023977932710763676, 0.2964736797698838])
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.sub_cb)
        self.markers = []
        self.vd = self.p_1 - self.p_0

    def sub_cb(self, msg):
        self.markers = []
        for marker in msg.markers:
            pos = marker.pose.pose.position
            p_l = np.array([pos.x, pos.y, pos.z])
            v_pl = np.array([pos.x, pos.y, pos.z]) - self.p_0
            dist = sqrt(
                (p_l[0] - self.p_0[0])**2 + \
                (p_l[1] - self.p_0[1])**2 + \
                (p_l[2] - self.p_0[2])**2
            )
            angle = angle_between(self.vd, v_pl)
            self.markers.append([marker.id, dist, angle])

    def get_markers(self):
        return self.markers

    def get_nearest_marker(self):
        min_idx = 999
        markers = []
        for i in range(100):
            time.sleep(0.01)
            markers += self.markers
        for index, m in enumerate(markers):
            if m[0] < min_idx:
                min_idx = index
        if min_idx >= 999:
            return []
        else:
            return markers[min_idx]

def thread_move_action():
    CMDcontrol.CMD_transfer()


def action(act_name):
    CMDcontrol.action_append(act_name)


def init_action_thread():
    th2 = threading.Thread(target=thread_move_action)
    th2.setDaemon(True)
    th2.start()

def stand(n=1):
    for _ in range(n):
        action('Stand')
        time.sleep(1)

def turn_left(n=1):
    for _ in range(n):
        action('turn001L')
        time.sleep(1)

def turn_right(n=1):
    for _ in range(n):
        action('turn001R')
        time.sleep(1)

def turn_left03(n=1):
    for _ in range(n):
        action('turn003L')
        time.sleep(1)

def turn_right03(n=1):
    for _ in range(n):
        action('turn003R')
        time.sleep(1)

def turn_left04(n=1):
    for _ in range(n):
        action('turn004L')
        time.sleep(1)

def turn_right04(n=1):
    for _ in range(n):
        action('turn004R')
        time.sleep(1)

def walk_slow00(n=1):
    for _ in range(n):
        action('Forwalk00')
        time.sleep(1)

def walk_slow(n=1):
    for _ in range(n):
        action('Forwalk01')
        time.sleep(1)

def walk_fast(n=1):
    for _ in range(n):
        action('Forwalk02')
        time.sleep(1)

def Back2Run(n=1):
    for _ in range(n):
        action('Back2Run')
        time.sleep(1)

def turn_right10(n=1):
    for _ in range(n):
        action('turn010R')
        time.sleep(2)

def turn_left10(n=1):
    for _ in range(n):
        action('turn010L')
        time.sleep(2)        


def UpBridge2(n=1):
    for _ in range(n):
        action('UpBridge2')
        time.sleep(5)

def turn_and_walk(dist, angle):
    # turn
    if (abs(angle) > 10) and (dist > 0.03):
        if angle < -10 and angle > -20:
            action('turn001L')
        if angle > 10 and angle < 20:
            action('turn001R')
        
        if angle < -20:
            if dist > 0.08:
                action('turn001L')
            else:
                action('Left02move')
        if angle > 20:
            if dist > 0.08:
                action('turn001R')
            else:
                action('Right02move')
        

    else:
        walk_slow()

    time.sleep(1)


def turn_once(dist, angle):
    is_turn_done = False
    if (abs(angle) > 10) and (dist > 0.03):
        if angle < -10 and angle > -20:
            action('turn001L')
        if angle > 10 and angle < 20:
            action('turn001R')

        if angle < -20:
            if dist > 0.1:
                action('turn001L')
            else:
                action('Left02move')
        if angle > 20:
            if dist > 0.1:
                action('turn001R')
            else:
                action('Right02move')
    else:
        is_turn_done = True
    return is_turn_done


def main():
    try:
        cur_tagid = 0
        step = 0
        
        rospy.init_node('ar_tag_tracker')
        tag = TagConverter()
        
        print('Start action thread ...')
        init_action_thread()
        time.sleep(1)

        stage = 'fastwalk'
        # stage = 'kick_ball'

        has_big_turn = False
        # stage = 'prepare_up_floor'

        # kick_ball(CMDcontrol)
        # exit(0)

        while not rospy.is_shutdown():
            time.sleep(0.1)
            
            marker = tag.get_nearest_marker()
            if len(marker) == 0:
                print('No marker found ! Stop?')

                if stage == 'fast_walk':
                    action('fastForward03')
                    time.sleep(3)
                    stage_fast_walk = False

                if stage == 'corner_turn':
                    print('转向对准楼梯Tag')
                    has_big_turn = True
                    walk_slow()
                    walk_slow()
                    time.sleep(2)
                    turn_left10()
                    time.sleep(5)
                
                if stage == 'prepare_up_floor':
                    print('准备上台阶')
                    # step1
                    action('turn001R')
                    walk_slow()
                    walk_slow00()
                    time.sleep(3)
                    action('turn001R')
                    action('UpBridge2')
                    walk_slow00()
                    time.sleep(6)
                    # step2
                    action('UpBridge2')
                    walk_slow00()
                    time.sleep(6)
                    # step3
                    action('UpBridge2')
                    time.sleep(6)
                    stage = 'kick_ball'
                if stage == 'kick_ball':
                    print('开始踢球')
                    time.sleep(3)
                    kick_ball(CMDcontrol)

                continue

            print(f'id={marker[0]}, dist={marker[1]}, angle={marker[2]}')

            if marker[0] <3:
                stage = 'fast_walk'
                is_turn_done = turn_once(marker[1], marker[2])
                if is_turn_done:
                    action('fastForward05')
                    time.sleep(3)
            else:
                stage_fast_walk = False
            
            if marker[0] == 3:
                stage = 'slow_walk'
                print('准备行走至转向位')
                turn_and_walk(marker[1], marker[2])

            if marker[0] == 4:
                stage = 'corner_turn'
                print('行走至转向位，然后转向')
                turn_and_walk(marker[1], marker[2])

            if marker[0] == 5:
                if not has_big_turn:
                    walk_slow()
                    walk_slow()
                    walk_slow()
                    time.sleep(2)
                    turn_left10()
                    time.sleep(5)
                    has_big_turn = True
                # follow once 到楼梯起始位
                # 往前挪几步贴住台阶
                # 执行上台阶动作1、2、3
                stage = 'prepare_up_floor'
                print('准备上台阶')
                turn_and_walk(marker[1], marker[2])

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()