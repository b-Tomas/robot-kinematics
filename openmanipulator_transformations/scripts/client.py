#!/usr/bin/env python3

import sys
import os
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.msg import JointPosition
from openmanipulator_transformations.srv import Transform

"""
JointPosition.msg:
string[]   joint_name
float64[]  position
float64    max_accelerations_scaling_factor
float64    max_velocity_scaling_factor

SetJointPosition.srv:
string planning_group
JointPosition joint_position
float64 path_time
---
bool is_planned
"""

ROBOT_CONTROL_SERVICE_NAME = "/goal_joint_space_path"
TRANSFORM_SERVICE_NAME = "/transformations/transform"


# This is a proof of concept script. It is not meant to be used for anything in particular
def control_robot(vec):
    rospy.wait_for_service(ROBOT_CONTROL_SERVICE_NAME)
    try:
        service = rospy.ServiceProxy(ROBOT_CONTROL_SERVICE_NAME, SetJointPosition)
        
        #  Transform position in space to joint angles

        req = SetJointPositionRequest()
        req.planning_group = "abc" # string
        req.joint_position  = JointPosition(["joint1", "joint2", "joint3", "joint4"], [vec[0], vec[1], vec[2], vec[3]], 1.0, 2.0) # JointPosition
        req.path_time = 1.0 # float64

        resp = service(req)

        print("[+] Comunicación con el robot exitosa.")

        return resp
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def input_position():
    # get position in space
    vec = []
    try:    
        vec.append(float(input("Pos. X: ")))
        vec.append(float(input("Pos. Y: ")))
        vec.append(float(input("Pos. Z: ")))
        return vec
    except ValueError:
        print("[!] Error al ingresar un valor")
        return 

def filtrar(vec):
    #Adjust positions within the range of -pi to pi for the robot's movement
    vec_list = list(vec)  # Convert tupla to list

    for i in range(3):
        while vec_list [i] > 3.15 or vec_list [i]< -3.15 : 
            if vec_list [i] > 3.15:
                vec_list[i] -= 6.28
            elif vec_list[i] < -3.15:
                vec_list[i] += 6.28

    vec_modified = tuple(vec_list)  # Convert list to tuple 
    return vec_modified


def transform_position(vec):
    rospy.wait_for_service(TRANSFORM_SERVICE_NAME)
    
    try:
        service = rospy.ServiceProxy(TRANSFORM_SERVICE_NAME, Transform)        
        
        resp = service(vec) # Transform x, y, z position to joint angles

        vec_joint = resp.joint_angles 

        filtrar(vec_joint) 
        
        return vec_joint
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}") 


def show_menu(): 
    print("""

 .----------------.  .----------------.  .----------------.  .----------------.  .----------------.   
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |  
| |  _______     | || |     ____     | || |   ______     | || |     ____     | || |  _________   | |  
| | |_   __ \    | || |   .'    `.   | || |  |_   _ \    | || |   .'    `.   | || | |  _   _  |  | |  
| |   | |__) |   | || |  /  .--.  \  | || |    | |_) |   | || |  /  .--.  \  | || | |_/ | | \_|  | |  
| |   |  __ /    | || |  | |    | |  | || |    |  __'.   | || |  | |    | |  | || |     | |      | |  
| |  _| |  \ \_  | || |  \  `--'  /  | || |   _| |__) |  | || |  \  `--'  /  | || |    _| |_     | |  
| | |____| |___| | || |   `.____.'   | || |  |_______/   | || |   `.____.'   | || |   |_____|    | |  
| |              | || |              | || |              | || |              | || |              | |  
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |  
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'   
 .----------------.  .----------------.  .----------------.  .----------------.  .----------------.   
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |  
| | ____    ____ | || |      __      | || |  _________   | || |  _________   | || |     ______   | |  
| ||_   \  /   _|| || |     /  \     | || | |  _   _  |  | || | |_   ___  |  | || |   .' ___  |  | |  
| |  |   \/   |  | || |    / /\ \    | || | |_/ | | \_|  | || |   | |_  \_|  | || |  / .'   \_|  | |  
| |  | |\  /| |  | || |   / ____ \   | || |     | |      | || |   |  _|  _   | || |  | |         | |  
| | _| |_\/_| |_ | || | _/ /    \ \_ | || |    _| |_     | || |  _| |___/ |  | || |  \ `.___.'\  | |  
| ||_____||_____|| || ||____|  |____|| || |   |_____|    | || | |_________|  | || |   `._____.'  | |  
| |              | || |              | || |              | || |              | || |              | |  
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |  
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'   

    Bienvenido al nodo Cliente, envía una posición en los ejes x, y, z a las que desea mover el robot.

    """)
    show_commands()

def show_commands():
    print("""
    
    Comando             Descripcion

    help    ?           Muestra este menu
    pos     position    Envio de posicion al robot
    home                Envio posicion (0, 0, 0)
    exit    q           Finalizar programa
    clc                 Limpiar consola

    Ejemplo de uso:
    > pos
        Pos. X: 1.8
        Pos. Y: 0.5
        Pos. Z: -0.2

    """)

def clc():
    if os.name == 'nt':  # to Windows
        os.system('cls')
    else:  # to Unix (Linux, macOS)
        os.system('clear')

if __name__ == "__main__": 
    clc()
    show_menu()

    while True:

        # Enter command
        command = input("\n> ")
        

        # Filter commands
        if command == 'pos' or command == 'position':
            # Enter position to send to the robot
            vec_position = input_position()
            
            if vec_position != None:
                vec_joint = transform_position(vec_position)
                if vec_joint != None:
                    control_robot(vec_joint)
                else:
                    print("[-] Error al transformar posicion.")
            
        elif command == 'home':
            # Send init pose
            print("[*] Envio posicion origen.")
            vec_joint = transform_position([0, 0, 0])
            if vec_joint != None:
                control_robot(vec_joint)
            else:
                print("[-] Error al transformar posicion origio.")

        elif command == 'help' or command == '?':
            # Show menu and usage example
            show_commands()
        
        elif command == 'clc':
            # clean console
            clc()
        
        elif command == 'exit' or command == 'q':
            # Finalize program
            break

        else: 
            print("[!] Comando ingresado no reconocido")