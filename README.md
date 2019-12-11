# CollaborativeUR3ROSETTA

## Repositorio para el proyecto de simulación de tareas colaborativas (ROSETTA lab - UNCuyo)

La demostración incluye las tareas siguientes, controladas por una máquina de estados:

- Recogida de pieza
- Manipulación sincronizada de pieza
- Manipulación conjunta de una caja

Para obtener una documentación e instrucciones sobre cómo instalar y usar este paquete, consulte el [wiki] (https://github.com/thibs-sigma/CollaborativeUR3ROSETTA/wiki).

A continuación se describen los pasos principales de instalación y funcionamiento.

## Video

![Global demo](04global_demo.gif)

<!-- [![Watch the video]()](https://youtu.be/jia6zDLpsHE) -->


## Dependencias (a instalar por separado, antes de compilar el proyecto)

- Ubuntu 16.04 LTS + ROS Kinetic
- MoveIt! (`sudo apt install ros-kinetic-moveit*`)
- Python (probado con Python 3.5.2 + 2.7.12, funcionando con Python 2.7.12)
- OpenCV (probado con `OpenCV 3.3.1-dev`)
- V-REP PRO EDU 3.6.2 (instrucciones y descargado http://www.coppeliarobotics.com/downloads.html)

## Instalación

- `git clone` ese repo en una carpeta personal (por ejemplo, `CooperationSimu_ws/src`)
- Compila con `catkin_make`
- Añade `source /home/YOUR_USERNAME/CooperationSimu_ws/devel/setup.bash` a tu archivo `.bashrc`

- VREP Installation : http://coppeliarobotics.com/previousVersions (3.6.2) + untar + añadir al bashrc
  - export VREP_ROOT=/home/thib/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04
  - export VREP_ROOT_DIR=/home/thib/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04
  - ./vrep.sh para verificar instalación
- *VREP ROS* --> seguir ese [![tutorial]()](http://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm)
- Cambiar *launch_vrep.sh*

## Lanzamiento

TO DO
