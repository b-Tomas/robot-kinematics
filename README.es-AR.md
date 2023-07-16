# Cinemática inversa y directa aplicada

<p align="center">
  <img src="docs/demo.gif" />
</p>

## Descripción

La cinemática de robots es el campo de investigación que estudia el movimiento de cadenas
articuladas con múltiples grados de libertad. Este proyecto es resultado de una propuesta
por parte de docentes del área de matemáticas que decidimos llevar al campo de la robótica
para demostrar una de las tantas aplicaciones del álgebra lineal, planteando un problema
cuya base es lograr el movimiento de las articulaciones de un brazo robótico empleando
transformaciones lineales.

Este proyecto puede ser utilizado de manera educativa tanto para demostrar los coneptos
matemáticos relacionados con coordenadas homogéneas y [parámetros de Denavit–Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
como para permitir un primer contacto con la robótica utilizando una simulación del robot
open-source [OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/).

<p align="center">
  <img src="docs/diagram_annotated.png" width="500" />
</p>

El trabajo de programación realizado consistió en, al entorno simulado de [OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/),
añadir dos componentes que implementan la solución matemática desarrollada:

- **`/transformations/transform`** ROS Service: Toma como parámetro una coordenada del
  espacio (x, y, z) y un ángulo de agarre, y utilizando las fórmulas de cinemática inversa
  desarrolladas calcula los parámetros del robot, o ánglos de las articulaciones, para que
  la posición de el _end-effector_ coincida con los parámetros dados.

- **`Client CLI`**: Es la interfaz que utilizará el usuario para interactuar con el servicio
  anterior y luego enviar los resultados al controlador del robot.

Estos componentes son parte del paquete de ROS `openmanipulator_transformations` provisto en este repositorio.

El desarrollo matemático del proceso se encuentra en el documento [`docs/Cinemática_de_robots.pdf`](https://github.com/b-Tomas/robot-kinematics/blob/main/docs/Cinemática_de_robots.pdf).

## Configuración del entorno y utilización

TODO: Either copy and translate the `Building the workspace` section from PR #22 or add a link to it.

Para lanzar la simulación, en diferentes paneles de tmux ejecute:

```sh
# Launch simulation, GUI and transformations server
roslaunch openmanipulator_transformations transformations.launch
# Launch the CLI
roslaunch openmanipulator_transformations cli.launch
```

Esto lanzará la simulación en Gazebo, la visualización en RViz, el controlador del robot y una
herramienta gráfica para ver y enviar parámetros del robot, además del servicio de transformaciones
y la CLI de este paquete, en la que podrá ejecutar el comando `?` para obtener información sobre su
utilización.

<p align="center">
  <img src="docs/cli.png" />
</p>

Antes de enviar una posición, inicie la simulación en Gazebo con el botón ▶️ y en la ventana
`OpenManipulator control GUI` haga click en el botón `Timer Start`.

Luego, envíe la posición objetivo (0.1, 0.1, 0.2, 0.0) (las unidades de distancia se encuentran en
metros y los ángulos en radianes) y observe en la ventana de control que la posición final del
_end-effector_ es la enviada.

## Autores

Este trabajo fue realizado por los estudiantes de [Ingeniería en Computación](https://ic.info.unlp.edu.ar/) de las facultades de Ingeniería e Informática de la [Universidad Nacional de La Plata](https://unlp.edu.ar):

- Tomás Badenes
- Juan Martín Seery
- Santiago Adriel Fernández
- Lorenzo Majoros
