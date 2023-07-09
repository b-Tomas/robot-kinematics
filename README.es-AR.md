# **Posicionamiento con Transformaciones Lineales**

## Descripción del proyecto 
Este proyecto es el resultado de una propuesta realizada por un profesor de Matemática para demostrar una de las tantas aplicaciones que tienen las transformaciones lineales.
Los desarrolladores del mismo, decidimos tomar la propuesta y llevarla al campo de la robótica, planteando un problema cuya base era lograr el movimiento de las articulaciones de un brazo robótico empleando dichas transformaciones.   
    
**Posicionamiento con Transformaciones Lineales** puede ser utilizado de manera educativa para un primer contacto con robots articulados y ver como el movimiento de los distintos rotores cambia los ejes de los demas. 

Para lograr que el robot realizara las acciones que se le manden a través de la consola, empleamos una libreria de frameworks para el desarrollo de software de robots que es facilitada por ROS.    

Se cuenta con un nodo cliente con una interfaz de usuario de tipo CLI que recibe datos ingresados por un usuario, estos se envían a un servidor que los procesa y retorna los ángulos de cada rotor del robot. 
Los angulos que retorna el primer servidor, son recibidos por el cliente, que los envía a un segundo servidor cuyo trabajo es comunicarse con las articulaciones del robot y lograr el movimiento de las mismas a la posición deseada.    
     
Para lograr el movimiento de las articulaciones, se aplicaron diferentes herramientas matemáticas que nos ayudaron a modelar el brazo.

## Listado de tecnologias utilizadas
- `Docker`
- `ROS`
- `Python`
## Setup e instalación

Para la instalación del simulador visite la sección correspondiente del [documento en inglés](https://github.com/b-Tomas/robot-kinematics/blob/main/README.md).

### Listado de comandos del CLI

|Comando             |Descripción                           |
|:-------------------|:-------------------------------------|
|`help`,     `?`     |Muestra este menú                     |
|`position`, `pos`   |Envio de posición al robot            |
|`home`              |Envio de posición original            |
|`verbose`,  `v`     |Activar o desactivar modo descriptivo |
|`exit`,     `q`     |Finalizar programa                    |
|`clc`               |Limpiar consola                       |
