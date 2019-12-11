# CollaborativeUR3ROSETTA

## Repositorio para el proyecto de simulación de tareas colaborativas (ROSETTA lab - UNCuyo)

La demostración incluye las tareas siguientes, controladas por una máquina de estados:

- Recogida de pieza
- Manipulación sincronizada de pieza
- Manipulación conjunta de una caja

Para obtener una documentación e instrucciones sobre cómo instalar y usar este paquete, consulte el [wiki](https://github.com/thibs-sigma/CollaborativeUR3ROSETTA/wiki).

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

## Instalación de V-REP

- Descargar V-REP 3.6.2 acá: http://coppeliarobotics.com/previousVersions (3.6.2) 
  - La versión actual de V-REP, ahora llamado CoppeliaSim, es la 4.0.0. No ha sido probado con ese proyecto. Así se aconseja descargar la ultima version de V-REP 3.6.2.
- Descomprimir el paquete, luego va en la carpeta descargada y ejecuta el script de installación
  - ```bash
    $ cd *CARPETA DESCARGADA*
    $ ./vrep.sh
- Luego añadir las lineas siguientes al archivo `.bashrc` (`gedit .bashrc` en la `home`)
  - `export VREP_ROOT=/home/**NOMBRE DE USUARIO**/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04`
  - `export VREP_ROOT_DIR=/home/**NOMBRE DE USUARIO**/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04`

- Para verificar la instalación, ejectua:
  - ```bash
    $ cd $VREP_ROOT
    $ ./vrep.sh
    ``` 

- Para la installación de *VREP ROS*, seguir ese [tutorial](http://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm)

## Creación del ROS Workspace

- `git clone` ese repo en tu carpeta personal (`home`)
- Compila con `catkin_make`
- Añade `source /home/**NOMBRE DE USUARIO**/CollaborativeUR3ROSETTA/devel/setup.bash` a tu archivo `.bashrc` (edición accessible con `gedit .bashrc` en la `home`)
- Luego copiar el archivo plugin de *ROS_Control* `libv_repExtRosControl.so` en la carpeta `CollaborativeUR3ROSETTA/devel/lib` y poner ese plugin en la carpeta raíz de V-REP: `/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04` por ejemplo


## Lanzamiento

Una vez todo compilado, ejecuta los comandos siguientes.

- Abre un Terminal, y ejecuta el comando siguiente:
  - `$ roslaunch cooperative_demo launch_ur3_simulation.launch`

- Ese único comando permite lanzar todos los nodos necesarios a la ejecución de la demostración (V-REP, MoveIt!, ros_control, etc.), incluyendo la interfaz de control de la simulación.

- Para acceder a la interfaz de control de la simulación:
  - Abre un navegador Web (preferiblemente Chrome o Firefox)
  - Aceda a la dirección siguiente: `localhost:8080`
