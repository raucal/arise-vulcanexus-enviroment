# Imagen base
FROM vulcanexus:iron-desktop-amd64

# Etiqueta para descripción
LABEL descripcion="ARISE ROS2 in vulcanexus Iron"

ARG USERNAME=cervera5R
ENV USER=${USERNAME}

# Establecer el directorio de trabajo dentro del contenedor
WORKDIR /home/${USERNAME}/arise-vulcanexus-enviroment/arise_ws

# Definir un volumen para el directorio arise_ws
VOLUME arise_ws

# # Los directorios que se copian tienen que estar dentro del directorio donde se creo la imagen
# COPY arise_ws/requirements.txt . 

# Actualizar los repositorios e instalar paquetes necesarios
RUN apt-get update && \
    apt-get install -y \
        ros-iron-rosbridge-server \
        iputils-ping \
        qtcreator \
        libespeak-dev \
        alsa-utils \
        ros-iron-ament-cmake \
        portaudio19-dev \
        ros-iron-librealsense2* \
        ros-iron-realsense2-* \
        ros-iron-depthai-ros \
        net-tools

# RUN /bin/bash -c "rm -r build/ install/ log/"
# # RUN colcon build

# Agregar comandos al archivo .bashrc para configurar el entorno
RUN echo "source /opt/vulcanexus/iron/setup.bash" >> ~/.bashrc && \
    echo "cd /home/${USERNAME}/arise-vulcanexus-enviroment/arise_ws" >> ~/.bashrc && \
    echo "source install/setup.bash" >> ~/.bashrc && \
    echo "alias ws='cd /home/${USERNAME}/arise_ws'" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=/opt/vulcanexus/iron/" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    
# Compilar el proyecto usando colcon
RUN /bin/bash -c "source ~/.bashrc"

COPY requirements.txt .
RUN pip install -r requirements.txt 

# Establecer variables de entorno para la visualización y el editor
ENV DISPLAY=:0 

# Exponer puertos utilizados por ROS (no es necesario para la ejecución del contenedor)
EXPOSE 9090 11311


