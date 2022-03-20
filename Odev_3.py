"""
Berkay Yildirim
"""

import matplotlib.pyplot as plt
import numpy as np

animation=True

def movement(x_start,y_start,x_goal,y_goal,theta_start,dt,x_rex,y_rec):
    while True:
        """
        Movement func for go to goal algorithm
        """  
        v_error_r,v_error_l,theta_error,u1,u2 = calculate_error(x_start,y_start,theta_start,x_goal,y_goal)

        x_n,y_n,theta_n,dt = differential_robot_two_wheel(v_error_r,v_error_l,theta_error,x_start,y_start,theta_start,dt)
        
        x_rec,y_rec=route_draw(x_n,y_n)

        x_start, y_start, theta_start = recursive_position(x_n,y_n,theta_n)    
        
        if animation:
            plt.cla()
            vehicle(x_n,y_n,theta_n, x_rec, y_rec) 
            
        if u1==0 and u2==0:
            break


def calculate_error(x_start,y_start,theta_start,x_goal,y_goal): 
    """
    This func calculate error.
    """
    u1=x_goal-x_start
    u2=y_goal-y_start
   
    v_error_r = np.sqrt((u1)**2+(u2)**2)
    v_error_l = np.sqrt((u1)**2+(u2)**2)

    theta_goal = np.arctan2(u2,u1)
    u3=theta_goal-theta_start
    theta_error = np.arctan2(np.sin(u3),np.cos(u3))
    return v_error_r,v_error_l,theta_error,u1,u2

def differential_robot_two_wheel(v_error_r,v_error_l,theta_error,x_start,y_start,theta_start,dt):
    """
    New pose of diff drive robot 
    """
    x_n = x_start + (1/2)*(P_V_R(v_error_r)+P_V_L(v_error_l))*np.cos(theta_start)*dt
    y_n = y_start + (1/2)*(P_V_R(v_error_r)+P_V_L(v_error_l))*np.sin(theta_start)*dt
    theta_n = theta_start+P_H(theta_error)*dt
    dt+=0.01
    return x_n,y_n,theta_n,dt

def route_draw(x_n,y_n):
    """
    The function keeps the locations of the robot in the list.
    """
    x_rec.append(x_n)
    y_rec.append(y_n)
    return x_rec,y_rec

def P_V_R(v_error_r):
    """ 
    P controller for right wheel
    """
    kd=0.5
    return v_error_r*kd

def P_V_L(v_error_l):
    """ 
    P controller for left wheel
    """
    kd=0.5
    return v_error_l*kd

def P_H(theta_error):
    """ 
    P controller for heading angle
    """
    Kh=2
    return theta_error*Kh


def recursive_position(x_n,y_n,theta_n):
    """
    Recursive position of robot.
    We assign the newly pose as the old pose.
    """
    x_start = x_n
    y_start = y_n
    theta_start = theta_n
    return x_start, y_start, theta_start


def vehicle(x_n, y_n, theta_n, x_rec, y_rec):  
    """
    The function simulate vehicle.
    """
    start_1,start_2,start_3 = vehicle_initial_position()

    T = transformation_matrix(x_n, y_n, theta_n) 

    p1,p2,p3 = matrix_mul(T,start_1,start_2,start_3)

    plot_vehicle(p1,p2,p3)

    plt.plot(x_rec, y_rec, ',r') # We draw robot's route
    
    #If ESC is pressed, the plotting screen is closed.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    
    plt.xlim(-5, 15)
    plt.ylim(-5, 25)
    
    plt.pause(dt)


def vehicle_initial_position ():
    """
    vehicle initial position function
    """

    start_1 = np.array([0.55, 0, 1])
    start_2 = np.array([-0.55, 0.3, 1])
    start_3 = np.array([-0.55, -0.3, 1])

    return start_1,start_2,start_3


def transformation_matrix(x_n, y_n, theta_n):
    """ 
    Transformation matrix of robot
    """
    return np.array([
        [np.cos(theta_n), -np.sin(theta_n), x_n],
        [np.sin(theta_n), np.cos(theta_n), y_n],
        [0, 0, 1]
    ])


def matrix_mul(T,start_1,start_2,start_3):
    """
    Matrix multiplication function of the initial position of the vehicle and the updated position.
    """
    p1 = np.matmul(T, start_1)
    p2 = np.matmul(T, start_2)
    p3 = np.matmul(T, start_3)

    return p1,p2,p3

def plot_vehicle(p1,p2,p3):
    """
    The function plot vehicle.
    """
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='green', marker='.', linestyle='-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color='green', marker='.', linestyle='-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color='green', marker='.', linestyle='-')
    
if __name__=='__main__':
    x_start=0
    y_start=0
    x_goal = 10
    y_goal = 20
    theta_start=0
    dt=0.01
    x_rec, y_rec = [], []
    
    movement(x_start,y_start,x_goal,y_goal,theta_start,dt,x_rec,y_rec)
