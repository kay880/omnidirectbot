#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import sys
import time
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

vel_msg = Twist()
vel_msg.linear.x = 0.0
vel_msg.angular.z = 0.0

car_yf = 0
car_xf = 0
car_y = 0
car_x = 0
yaw = 0

P = 5.5
D = 4.5
I = 0.1

err_angle = 0
last_err_angle = 0
last_last_err_angle = 0

err_vx = 0
last_err_vx = 0
last_last_err_vx = 0
err_vy = 0
last_err_vy = 0
last_last_err_vy = 0

pcar_x = []
pcar_y = []
def location_callback(msg):
    global car_x, car_y, yaw, car_xf, car_yf
    car_x = int(msg.pose.position.x *1000)
    car_y = int(msg.pose.position.y *1000)
    car_xf = msg.pose.position.x
    car_yf = msg.pose.position.y
    orientation = msg.pose.orientation
    #将四元数转换为欧拉角
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    yaw = int(math.degrees(yaw))
    # print "car_x:", car_x, "\ncar_y:", car_y, "\nyaw:", yaw
    # print("----")
    # print "robot_x:", robot_x, "\nrobot_y:", robot_y
    # print "relative_x:", relative_x, "\nrelative_y:", relative_y
    # print "relative_x * width_scale:", relative_x * width_scale, "\nwidth_scale:", width_scale
    # num_args = len(args)
    # print(num_args)
# vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

def generate_sine_curve(amplitude, frequency, phase_shift, num_points=30):
    """
    生成正弦曲线的函数

    参数:
        amplitude: 正弦曲线的振幅
        frequency: 正弦曲线的频率
        phase_shift: 正弦曲线的相移
        num_points: 生成曲线上的点的数量，默认为1000

    返回:
        x_values: x轴上的点坐标
        y_values: y轴上的点坐标
    """
    x_values = np.linspace(0, np.pi, num_points)
    y_values = amplitude * np.sin(frequency * x_values + phase_shift)
    return x_values+0.807, y_values+0.821


def control_car_speed(linear_speed, angular_speed):
    
    max_x = 10
    max_a = 1
    if linear_speed > max_x:
        linear_speed = max_x
    if linear_speed < -max_x:
        linear_speed = -max_x
    if angular_speed < -max_a:
        angular_speed = -max_a
    if angular_speed > max_a:
        angular_speed = max_a

    vel_msg.linear.x = linear_speed  # 设置线速度
    vel_msg.angular.z = angular_speed  # 设置角速度
    print("vel_msg.linear.x",vel_msg.linear.x)
    print("vel_msg.angular.z",vel_msg.angular.z)
    vel_pub.publish(vel_msg)
    

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        # PID计算        
        global last_err_angle
        global last_last_err_angle
        last_last_err_angle = last_err_angle
        last_err_angle = error
        I_all = error + last_err_angle + last_last_err_angle
        D_all = error - 2 * last_err_angle + last_last_err_angle
        output = P * error - D * D_all + I * I_all
        
        # self.integral += error
        # derivative = error - self.prev_error
        # self.prev_error = error
        # output = self.kp * error + self.ki * self.integral - self.kd * derivative
        output = 0.001 * output
        max = 1
        if output > max:
            output = max
        if output < -max:
            output = -max
        return output
def get_angle_err(num):
    amplitude = 0.5
    frequency = 1.0
    phase_shift = 0.0
    global yaw
    x_values, y_values = generate_sine_curve(amplitude, frequency, phase_shift)
    x_values = np.round(x_values * 1000).astype(int)
    y_values = np.round(y_values * 1000).astype(int)
    # print(x_values)
    # print(y_values)
    # target_goal_list = [[505,389],[496,1468],[1973,1296],[1909,361]]
    err_x = x_values[num] - car_x
    err_y = y_values[num] - car_y
    dis = math.sqrt((err_x)**2 + (err_y)**2)
    angle_rad = math.atan2(err_y, err_x)
    target_angle = math.degrees(angle_rad)
    if 90 <= yaw <= 180 and -180 <= target_angle <= -160:
        target_angle = target_angle + 360
    if 160 <= target_angle <= 180 and -180 <= yaw <= -90: #
        yaw = yaw + 360
    angle_error =  target_angle - yaw
    # if abs(angle_error) > 340:
    #     angle_errortemp = 360 - abs(angle_error)
    #     angle_error = -angle_errortemp if angle_error > 0 else angle_errortemp 
    # print "target_angle", target_angle, "\nyaw", yaw, "\nangle_error", angle_error
    return err_x, err_y,dis,angle_rad


    
def update_car_position():
    amplitude = 0.5
    frequency = 1.0
    phase_shift = 0.0
    def update(frame):
        x, y = car_xf, car_yf

        # 模拟小车的实时位置更新，你可以根据实际情况从传感器或其他方式获取小车的位置
        pcar_x.append(x)
        pcar_y.append(y)
        # 更新小车的位置
        car_marker.set_data(pcar_x, pcar_y)

        return car_marker,
    # 生成正弦曲线
    x_values, y_values = generate_sine_curve(amplitude, frequency, phase_shift)
    fig, ax = plt.subplots(figsize=(8, 6))
    line, = ax.plot(x_values, y_values, label="Sine Curve")
    car_marker, = ax.plot(car_xf, car_yf, 'ro', markersize=10, label="Car Position")
    ani = FuncAnimation(fig, update, frames=np.linspace(0, np.pi, 100), interval=50, blit=True)
    ax.set_xlim(0, 3)  # 设置 x 轴范围从 0 到 10
    ax.set_ylim(0, 1.5)  # 设置 y 轴范围从 -1.5 到 1.5
    plt.show()
    
    
    
if __name__ == '__main__':
    rospy.init_node('car_control_node')  # 初始化ROS节点
    rospy.Subscriber('/vrpn_client_node/silun1/pose', PoseStamped, location_callback,queue_size=10)
    
    animation_thread = threading.Thread(target=update_car_position)
    animation_thread.start()

    rate = rospy.Rate(8)
    num = 0
    err_x, err_y,dis,angle_rad = get_angle_err(num)
    while not rospy.is_shutdown():
        print(num)
        while not rospy.is_shutdown():
            err_x, err_y,dis,angle_rad = get_angle_err(num)
            if angle_rad == 0:
                num += 1
                break
            dx = dis * math.cos(angle_rad)
            dy = dis * math.sin(angle_rad)
            
            print("angle_rad",angle_rad)
            # print("sin",math.sin(angle_rad),"cos",math.cos(angle_rad))
            last_last_err_vx = last_err_vx
            last_err_vx = err_vx
            err_vx = dx

            I_vx = err_vx + last_err_vx + last_last_err_vx
            D_vx = err_vx - 2*last_err_vx + last_last_err_vx
            PP_vx = 0.0005
            II_vx = 0
            DD_vx =0.00025
            vx = PP_vx * err_vx + I_vx * II_vx + DD_vx * D_vx
            
            last_last_err_vy = last_err_vy
            last_err_vy = err_vy
            err_vy = dy
            
            I_vy = err_vy + last_err_vy + last_last_err_vy
            D_vy = err_vy - 2*last_err_vy + last_last_err_vy
            PP_vy = 0.0005
            II_vy = 0
            DD_vy = 0.00035
            vy = PP_vy * err_vy + I_vy * II_vy + DD_vy* D_vy
            # print("dx",dx,"dy",dy,"vx",vx,"vy",vy)
            if vx >= 0.05:
                vx = 0.05
            if vx <= -0.05:
                vx = -0.05
            if vy >= 0.05:
                vy = 0.05
            if vy <= -0.05:
                vy = -0.05
            vx1 = "{:.4f}".format(vx)
            vy1 = "{:.4f}".format(vy)
            print("vx",vx1,"vy",vy1)
            if -50 < err_x < 50 and -50 < err_y < 50:
                # print("err_x",err_x,"err_y",err_y)
                vel_msg.linear.x = vx  # 设置线速度
                vel_msg.linear.y = vy
                vel_pub.publish(vel_msg)
                # print("have arrive goal--------------------------------------------------------------",num)
                num += 1
                if num > 29:
                    num = 0

                # print("break")
                break
            else:
                vel_msg.linear.x = vx  # 设置线速度
                vel_msg.linear.y = vy
                vel_pub.publish(vel_msg)
            rate.sleep()
    


