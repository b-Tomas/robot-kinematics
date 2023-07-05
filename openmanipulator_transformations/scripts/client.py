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
        
        # hacer Transformacion de posicion en el eslpacio a angulo de articulaciones

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
    # Se pide la posicion en el espacio
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
    #como el robot recibe posiciones de -pi a pi nos aseguramos de trasladarlo en el caso de que sea mayor al rango establecido para el movimiento del robot
    for i in range(0, 3):
        while vec [i] > 3.15 or vec [i]< -3.15 : 
            if vec [i] > 3.15:
                vec[i] -= 6.28
            elif vec[i] < -3.15:
                vec[i] += 6.28

def transform_position(vec):
    rospy.wait_for_service(TRANSFORM_SERVICE_NAME)
    
    try:
        service = rospy.ServiceProxy(TRANSFORM_SERVICE_NAME, Transform)        
        
        resp = service(vec) # transforma posicion x y z a angulos de articulaciones

        vec_joint = resp.joint_angles 

        filtrar(vec_joint) 
        
        return vec_joint
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}") 


def show_menu(): 
    print("""

    BANNER

    Bienvenido al nodo Cliente, envía una posición en los ejes x, y, z a las que desea mover el robot.

    """)
    show_commands()

def show_commands():
    print("""
    
    Command             Description

    help    ?           Show this menu
    pos     position    Send position to robot
    exit                Finalize program
    clc                 Clean console

    Ejemplo de uso:
    > pos
        Pos. X: 1.8
        Pos. Y: 0.5
        Pos. Z: -0.2

    """)

def clc():
    if os.name == 'nt':  # Para sistemas Windows
        os.system('cls')
    else:  # Para sistemas basados en Unix (Linux, macOS)
        os.system('clear')

if __name__ == "__main__": 
    clc()
    show_menu()

    while True:

        # Ingresar comando
        command = input("> ")
        

        # filtrar comando
        if command == 'pos' or command == 'position':
            # Ingresar posicion para enviar al robot
            vec_position = input_position()
            
            if vec_position != None:
                vec_joint = transform_position(vec_position)
                if vec_joint != None:
                    control_robot(vec_joint)
                else:
                    print("[-] Error al transformar posicion.")
            
        elif command == 'help' or command == '?':
            # Mostrar menu y ejemplo de uso
            show_commands()
        
        elif command == 'clc':
            # Limpiar consola
            clc()
        
        elif command == 'exit':
            # Finalizar programa
            break

        else: 
            print("[!] Comando ingresado no reconocido")
