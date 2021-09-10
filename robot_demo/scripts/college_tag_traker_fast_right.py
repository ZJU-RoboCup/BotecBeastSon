#!/usr/bin/env python
# -*- coding: utf-8 -*-
import CMDcontrol
import rospy
import tf
import time
import threading
import numpy as np
import math
from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from kickBallOnly import kick_ball

step_move_distance = 0.12


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
            quat = marker.pose.pose.orientation

            # print(marker)

            rpy = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            rpy_arc = [0, 0, 0]
            for i in range(len(rpy)):
                rpy_arc[i] = rpy[i] / math.pi * 180
            
            # print(rpy_arc)

            p_l = np.array([pos.x, pos.y, pos.z])
            v_pl = np.array([pos.x, pos.y, pos.z]) - self.p_0
            dist = sqrt(
                (p_l[0] - self.p_0[0])**2 + \
                (p_l[1] - self.p_0[1])**2 + \
                (p_l[2] - self.p_0[2])**2
            )
            angle = angle_between(self.vd, v_pl)
            self.markers.append([marker.id, dist, angle, rpy_arc[1]])

    def get_markers(self):
        return self.markers

    def get_nearest_marker(self):
        min_id = 999
        min_idx = 0
        markers = []
        for i in range(50):
            time.sleep(0.01)
            markers += self.markers
        
        for index, m in enumerate(markers):
            if m[0] < min_id:
                min_idx = index
                min_id = m[0]
        if min_id == 999:
            return []
        else:
            return markers[min_idx]

def thread_move_action():
    CMDcontrol.CMD_transfer()


def action(act_name):
    print(f'执行动作: {act_name}')
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
        # time.sleep(1)

def walk_fast(n=1):
    for _ in range(n):
        action('Forwalk02')
        # time.sleep(1)

def Back2Run(n=1):
    for _ in range(n):
        action('Back2Run')
        # time.sleep(1)

def turn_right10(n=1):
    for _ in range(n):
        action('turn010R')
        # time.sleep(2)

def UpBridge2(n=1):
    for _ in range(n):
        action('UpBridge2')
        time.sleep(1)

def turn_once(dist, angle, side_threshold=0.1):
    dist_side = dist * math.sin(abs(angle)*(math.pi/180))
    is_turn_done = False
    # action('Stand')
    if (abs(angle) > 7) and (dist > 0.03):
        if dist_side > side_threshold:
            action('Left02move') if angle < 0 else action('Right02move')
        else:
            action('turn001L') if angle < 0 else action('turn001R')
    else:
        is_turn_done = True
    return is_turn_done

def turn_once_fast(dist, angle, side_threshold=0.1):
    dist_side = dist * math.sin(abs(angle)*(math.pi/180))
    is_turn_done = False

    if (abs(angle) > 7) and (dist > 0.03):
        if dist_side > side_threshold:
            if dist_side < 0.05:
                action('Left02move') if angle < 0 else action('Right02move')
            else:
                action('Left3move') if angle < 0 else action('Right3move')
        else:
            action('turn001L') if angle < 0 else action('turn001R')
    else:
        is_turn_done = True
    return is_turn_done

def turn_once_corner(dist, angle, side_threshold=0.1):
    dist_side = dist * math.sin(abs(angle)*(math.pi/180))
    is_turn_done = False 
    
    if (abs(angle) > 8) and (dist > 0.03):
        if dist_side > side_threshold:
            action('Left02move') if angle < 0 else action('Right02move')
        else:
            if angle < 0:
                if -15 < angle < 0:
                    action('turn001L')
                if -30 < angle < -15:
                    action('turn003L')
                if angle < -30:
                    action('turn004L')
            if angle > 0:
                if 0 < angle < 15:
                    action('turn001R') 
                if 15 < angle < 30:
                    action('turn003R') 
                if angle > 30:
                    action('turn004R') 
    else:
        is_turn_done = True
    return is_turn_done
            

def turn_and_walk(dist, angle, side_threshold=0.1):
    is_turn_done = turn_once(dist, angle, side_threshold=side_threshold)
    if is_turn_done:
        walk_slow()


def turn_and_walk_corner(dist, angle):
    is_turn_done = turn_once_corner(dist, angle)
    if is_turn_done:
        walk_slow()
        

def main():
    try:
        rospy.init_node('ar_tag_tracker')
        tag = TagConverter()
        
        print('Start action thread ...')
        init_action_thread()
        time.sleep(1)

        stage = 'fast_walk'
        # stage = 'kick_ball'

        has_big_turn = False
        # stage = 'prepare_up_floor'

        # kick_ball(CMDcontrol)
        # exit(0)

        body_is_alignment = False
        body_is_alignment_ready = False


        while not rospy.is_shutdown():
            time.sleep(0.1)
            
            marker = tag.get_nearest_marker()
            

            if len(marker) == 0:
                print('No marker found ! Stop?')

                if stage == 'fast_walk':
                    action('fastForward03')
                    stage = 'fast_walk_end'

                if stage == 'fast_walk_end':
                    print('快走结束，但没有找到二维码？后退1步')
                    action('Back1Run')

                if stage == 'bridge_start':
                    print('过桥开始')

                if stage == 'bridge_end':
                    print('过桥结束')

                if stage == 'corner_turn':
                    print('看不到6，转向对准楼梯Tag')
                    has_big_turn = True
                    for i in range(0):
                        action('forwardSlow0403')
                    time.sleep(2)
                    action('turn010R')
                    time.sleep(1)
                    # stage = 'prepare_up_floor'
                
                if stage == 'prepare_up_floor':
                    print('准备上台阶')
                    # step1
                    # action('turn005R')
                    action('turn001R')
                    # action('turn001R')
                    for i in range(1):
                        walk_slow()
                    
                    action('UpBridge2')
                    walk_slow00()
                    
                    # step2
                    action('UpBridge2')
                    walk_slow00()

                    # step3
                    action('UpBridge2')
                    time.sleep(1)
                    stage = 'kick_ball'
                if stage == 'kick_ball':
                    print('开始踢球')
                    time.sleep(1)
                    kick_ball(CMDcontrol)

                continue

            
            theta = marker[3]
            dist_v = marker[1] * math.cos(abs(marker[2])*(math.pi/180))
            dist_h = marker[1] * math.sin(abs(marker[2])*(math.pi/180))

            print(f'id={marker[0]}, dist={marker[1]}, angle={marker[2]}, theta={marker[3]}, dist_v={dist_v}, dist_h={dist_h}')
            # continue
            # 快走阶段
            if marker[0] < 3 and stage == 'fast_walk':
                stage = 'fast_walk'

                if marker[0] == 0:
                    action('fastForward05')
                    continue

                is_turn_done = turn_once_fast(marker[1], marker[2], side_threshold=0.03)
                if is_turn_done:
                    if marker[0] == 2:
                        action('fastForward03')
                    else:
                        action('fastForward05')
                    
            
            # 准备行走到窄桥起点，对齐二维码执行快走
            if marker[0] == 3:
                in_circle_theta = 2
                in_circle_angle = 5
                in_circle_dist = 0.03
                stage = 'bridge_start'

                if dist_h > 0.10:
                    if theta < 0:
                        if abs(theta) > 30:
                            action('turn004L')
                        else:
                            action('turn001L')
                        
                        if dist_h < 0.05:
                            action('Left02move')
                        else:
                            action('Left3move')
                    else: 
                        if abs(theta) > 30:
                            action('turn004R')
                        else:
                            action('turn001R')
                        if dist_h < 0.05:
                            action('Right02move')
                        else:
                            action('Right3move')
                    continue
                    
                if abs(theta) > in_circle_theta and not body_is_alignment:
                    if theta < 0:
                        if abs(theta) > 30:
                            action('turn004L')
                        else:
                            action('turn001L')
                    else: 
                        if abs(theta) > 30:
                            action('turn004R')
                        else:
                            action('turn001R')
                else:
                    # print('已平行，开始侧移')
                    if abs(dist_h) > in_circle_dist:
                        print('侧移对齐')
                        if marker[2] < 0:
                            action('Left02move')
                        else:
                            action('Right02move')
                    else:
                        body_is_alignment = True
                
                if (dist_h < in_circle_dist) and (abs(theta) < in_circle_theta):
                    print(dist_h, theta)
                    body_is_alignment = True
                    body_is_alignment_ready = True
                else:
                    body_is_alignment = False
                    body_is_alignment_ready = False

                if dist_v < 0.05 and abs(theta) > 3 and not body_is_alignment_ready:
                    action('Back2Run')

                if body_is_alignment_ready:
                    print('快走过桥')
                    action('turn001R')
                    action('turn001R')
                    action('fastForward05')

            # 窄桥中间
            if marker[0] == 4:
                stage = 'bridge_middle'
                print('行走至独木桥中间')
                turn_and_walk(marker[1], marker[2], side_threshold=0.03)

            # 窄桥终点
            if marker[0] == 5:
                stage = 'bridge_end'
                print('行走至独木桥终点')

                is_turn_done = turn_once(marker[1], marker[2], side_threshold=0.05)
                if is_turn_done:
                    action('fastForward03')

            if marker[0] == 6:
                stage = 'corner_turn'
                print('行走至转向位，然后转向')
                is_turn_done = turn_once_corner(marker[1], marker[2], side_threshold=0.04)
                if is_turn_done:
                    action('forwardSlow0403')

            if marker[0] == 7:
                if not has_big_turn:
                    print('看到了 7，还没转弯')
                    for i in range(0):
                        action('forwardSlow0403')
                    turn_right10()
                    time.sleep(3)
                    has_big_turn = True
                # follow once 到楼梯起始位
                # 往前挪几步贴住台阶
                # 执行上台阶动作1、2、3
                stage = 'prepare_up_floor'
                print('准备上台阶')
                # turn_and_walk_corner(marker[1], marker[2])
                is_turn_done = turn_once_corner(marker[1], marker[2], side_threshold=0.04)
                if is_turn_done:
                    action('forwardSlow0403')


    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()