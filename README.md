# Entorno ROS2 ARISE

Este repositorio contiene la información y los archivos necesarios para poder poner en marcha los challenge 1 y 2 en un entorno de ROS2. 

# Índice

- [Instalación en Ubuntu 22.04 Nativo](#instalación-en-ubuntu-2204-nativo)
- [Modificar el .bashrc (opcional)](#modificar-el-bashrc-opcional)
- [Dependencias](#dependencias)
- [Ejecución del challenge 1 (Battery Disassembly)](#ejecución-del-challenge-1-battery-disassembly)
  - [Opción 1) Arranque de los launchs](#opción-1-arranque-de-los-launchs)
  - [Opción 2) Arranque uno a uno](#opción-2-arranque-uno-a-uno)
  - [Opción 3) Arranque mediante interfaz](#opción-3-arranque-mediante-interfaz)
- [Ejecución del challenge 2 (Fruit Picking)](#ejecución-del-challenge-2-fruit-picking)
  - [Opción 1) Arranque de los launchs](#opción-1-arranque-de-los-launchs-1)
  - [Opción 2) Arranque uno a uno](#opción-2-arranque-uno-a-uno-1)
  - [Opción 3) Arranque mediante interfaz](#opción-3-arranque-mediante-interfaz-1)
- [Instalación en Contenedor Docker](#instalación-en-contenedor-docker)

## Instalación en Ubuntu 22.04 Nativo

Se parte de que ya se tiene instalado ROS2 Humble con o sin Vulcanexus en una distribución de Ubuntu 22.04

En el directorio /home/$USER clonar el repositorio:

```bash
  cd /home/$USER
  git clone https://venus.cartif.es/arise_framework/arise-vulcanexus-enviroment
```

Instalar las librerías de python:
```bash
  cd arise-vulcanexus-enviroment/
  pip install -r requirements.txt
```

Ir al espacio de trabajo

```bash
  cd arise_ws/
```

Compilar

```bash
  colcon build
```

Activar el entorno actual

```bash
  source install/setup.bash
```
## Modificar el .bashrc (opcional)
El archivo .bashrc se ejecuta siempre cuando se abre una nueva terminal en Ubuntu. Para hacer el desarrollo más cómodo y ágil recomiendo añadir a este archivos los comandos encargados de cargar el entorno de ROS2 y del directorio de trabajo, un alias "ws" para dirigirse al directorio de trabajo y la tabulación para "colcon". Introducir en una terminal los siguientes comandos:

```bash
  echo "source /opt/vulcanexus/iron/setup.bash" >> ~/.bashrc
  echo "cd /home/$USER/arise-vulcanexus-enviroment/arise_ws" >> ~/.bashrc
  echo "source install/setup.bash" >> ~/.bashrc
  echo "alias ws='cd /home/$USER/arise-vulcanexus-enviroment/arise_ws'" >> ~/.bashrc
  echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
  echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
  source ~/.bashrc
```
## Dependencias
Se deben de instalar una serie de paquetes con apt install:

```bash
apt install ros-iron-rosbridge-server \
            libespeak-dev \
            alsa-utils \
            ros-iron-ament-cmake \
            ros-iron-librealsense2* \
            ros-iron-ur \
            ros-iron-realsense2-* \
            portaudio19-dev \
            ros-iron-depthai-ros
            librealsense2-dkms \
            librealsense2-utils \
            net-tools
```

## Ejecución del challenge 1 (Battery Disassembly)
Configurar el robot en modo remoto con el TCP (0,0,0) activo.

#### Opción 1) Arranque de los launchs

Nodo cámara: 

Opción 1 (Intel Real Sense D435):

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30 enable_sync:=true align_depth.enable:=true
```
Opción 2 (Luxonis):

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py
```

Nodo puente websocket:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```

Nodo aplicación:

```bash
ros2 launch battery_disassembly battery_disassembly.launch.py
```

#### Opción 2) Arranque uno a uno

Ejecutar el servidor websocket:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```

Ejecutar el nodo de detección:

```bash
ros2 run battery_disassembly detect_screws 
```

Ejecutar el nodo de interfaz de sonido:

```bash
ros2 run battery_disassembly audio_interface 
```

Ejecutar el nodo de comunicacón HRI (Hololens2-UR5e):

```bash
ros2 run battery_disassembly hri_communication 
```

Ejecutar el nodo de la cámara:

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30 enable_sync:=true align_depth.enable:=true
```

#### Opción 3) Arranque mediante interfaz
Se debe de ejecutar el programa **interface_ch1.py** para abrir una interfaz que permita ejecutar los diferentes nodos involucrados en el challenge. Cuando arranquemos un nodo, se abrirá una interfaz que ejecuta los comandos de ubuntu necesarios para ejecutar el nodo correspondiente. En ese momento se habilitará el botón Detener, que permite detener el proceso correspondiente con señal SIGINT(Crtl +C). Existe la posibilidad de ejecutar todos los nodos a la vez. Esto consigue abrir una terminal diferente para cada nodo. Cuando se pulse al botoón "Matar todos los procesos y salir",se pararán todos los procesos con señal SIGINT y se cerrarán todas las terminales.


## Ejecución del challenge 2 (Fruit Picking)
Configurar el robot en modo remoto con el TCP (0,0,0) activo.

#### Opción 1) Arranque de los launchs

Nodo aplicación:

```bash
ros2 launch fruit_picking fruit_picking.launch.py
```

Nodo cámara:

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=d435 depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 enable_sync:=true align_depth.enable:=true 
```

#### Opción 2) Arranque uno a uno
Ejecutar el nodo de detección:

```bash
ros2 run fruit_picking detect_fruits
```

Ejecutar el nodo del UR10e:

```bash
ros2 run fruit_picking robot_pick 
```

Ejecutar el nodo de interfaz de sonido:

```bash
ros2 run battery_disassembly audio_interface 
```

Ejecutar el servidor de píxeles a 3D:

```bash
ros2 run fruit_picking intel_transforms 
```

Ejecutar el nodo de comunicación HRI (Ollama-UR10e):

```bash
ros2 run fruit_picking fruits_manager 
```

Ejecutar el nodo de la cámara:

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=d435 depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 enable_sync:=true align_depth.enable:=true 
```

Ejecutar el nodo de reconocimiento de voz:

```bash
ros2 run fruit_picking arise_ai 
```

#### Opción 3) Arranque mediante interfaz
Se debe de ejecutar el programa **interface_ch2.py** para abrir una interfaz que permita ejecutar los diferentes nodos involucrados en el challenge. Cuando arranquemos un nodo, se abrirá una interfaz que ejecuta los comandos de ubuntu necesarios para ejecutar el nodo correspondiente. En ese momento se habilitará el botón Detener, que permite detener el proceso correspondiente con señal SIGINT(Crtl +C). Existe la posibilidad de ejecutar todos los nodos a la vez. Esto consigue abrir una terminal diferente para cada nodo. Cuando se pulse al botoón "Matar todos los procesos y salir",se pararán todos los procesos con señal SIGINT y se cerrarán todas las terminales.


## Instalación en Contenedor Docker
Utilizar la imagen de Vulcanexus-Iron. Se puede descargar de la página oficial: https://vulcanexus.org/download

Cargar la imagen de Vulcanexus Iron: 

```bash
docker load -i vulcanexus-iron-desktop-amd64-3.3.0.tar.gz
```

Cargar la imagen del Dockerfile desde el directorio donde se encuentra: 

```bash
cd /home/$USER/arise-vulcanexus-enviroment
docker build -t arise_vulcanexus:iron .
```

Construir contenedor con la imagen de Docker (Ubuntu):

```bash
docker run -it -v /dev/:/dev/ --privileged -e DISPLAY=$DISPLAY --gpus all --name arise_challenges --device-cgroup-rule='c 189:* rmw' -p 9090:9090 -p 11311:11311 -v /home/$USER/arise-vulcanexus-enviroment/arise_ws:/home/$USER/arise-vulcanexus-enviroment/arise_ws -v /tmp/.X11-unix:/tmp/.X11-unix arise_vulcanexus:iron
```
