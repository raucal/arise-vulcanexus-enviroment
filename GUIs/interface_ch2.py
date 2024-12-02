#!/usr/bin/env python3

import customtkinter as ctk
import subprocess
import time
import os
user = os.getenv('USER')

# Configuración de la interfaz con customtkinter
ctk.set_appearance_mode("dark")  # Opciones: "System", "Dark", "Light"
ctk.set_default_color_theme("blue")  # Opciones: "blue", "dark-blue", "green"

# Crear la ventana principal
root = ctk.CTk()
root.title("Interfaz de control CH2 - ARISE")
root.geometry("500x650")  # Ajustar el tamaño de la ventana para acomodar el nuevo botón

# Diccionario para guardar procesos iniciados
procesos = {}

load_ws = "cd /home/"+user+"/arise-vulcanexus-enviroment/arise_ws && source install/setup.bash"

def ejecutar_comando(command, button_start, button_stop, key, new_terminal):
    if new_terminal:
        # Ejecutar el comando en una nueva terminal
        proceso = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command + "; exec bash"], preexec_fn=os.setsid)
        procesos[key] = proceso
        # Deshabilitar el botón de inicio y cambiar su color a gris
        button_start.configure(state="disabled", fg_color="gray")
        # Habilitar el botón de detener y cambiar su color a rojo
        button_stop.configure(state="normal", fg_color="red")
    else:
        # Ejecutar el comando en segundo plano
        subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

def detener_comando(command, button_start, button_stop, key):
    # Ejecutar el comando para detener el proceso
    subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
    # Habilitar el botón de inicio y cambiar su color a azul
    button_start.configure(state="normal", fg_color="blue")
    # Deshabilitar el botón de detener y cambiar su color a gris
    button_stop.configure(state="disabled", fg_color="gray")
    # Eliminar el proceso del diccionario
    if key in procesos:
        procesos.pop(key)

def ejecutar_comando1():
    ejecutar_comando(
        load_ws + " && ros2 run fruit_picking detect_fruits",
        btn_comando1, btn_comando1d, "detect_fruits", True)

def detener_comando1():
    detener_comando("pkill -SIGINT -f detect_fruits", btn_comando1, btn_comando1d, "detect_fruits")

def ejecutar_comando2():
    ejecutar_comando(
        load_ws + " && ros2 run fruit_picking robot_pick",
        btn_comando2, btn_comando2d, "robot_pick", True)

def detener_comando2():
    detener_comando("pkill -SIGINT -f robot_pick", btn_comando2, btn_comando2d, "robot_pick")

def ejecutar_comando3():
    ejecutar_comando(
        load_ws + " && ros2 run battery_disassembly audio_interface",
        btn_comando3, btn_comando3d, "audio_interface", True)

def detener_comando3():
    detener_comando("pkill -SIGINT -f audio_interface", btn_comando3, btn_comando3d, "audio_interface")

def ejecutar_comando4():
    ejecutar_comando(
        load_ws + " && ros2 run fruit_picking intel_transforms",
        btn_comando4, btn_comando4d, "intel_transforms", True)

def detener_comando4():
    detener_comando("pkill -SIGINT -f intel_transforms", btn_comando4, btn_comando4d, "intel_transforms")

def ejecutar_comando5():
    ejecutar_comando(
        load_ws + " && ros2 run fruit_picking fruits_manager",
        btn_comando5, btn_comando5d, "fruits_manager", True)

def detener_comando5():
    detener_comando("pkill -SIGINT -f fruits_manager", btn_comando5, btn_comando5d, "fruits_manager")

def ejecutar_comando6():
    ejecutar_comando(
        load_ws + " && ros2 launch realsense2_camera rs_launch.py camera_namespace:=d435 depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 enable_sync:=true align_depth.enable:=true",
        btn_comando6, btn_comando6d, "rs_launch.py", True)

def detener_comando6():
    detener_comando("pkill -SIGINT -f rs_launch.py", btn_comando6, btn_comando6d, "rs_launch.py")

def ejecutar_comando7():
    ejecutar_comando(
        load_ws + " && ros2 run fruit_picking arise_ai",
        btn_comando7, btn_comando7d, "arise_ai", True)

def detener_comando7():
    detener_comando("pkill -SIGINT -f arise_ai", btn_comando7, btn_comando7d, "CV2_hololens_stream")

def ejecutar_todos_los_comandos():
    # Ejecutar todos los comandos individualmente
    ejecutar_comando1()
    ejecutar_comando2()
    ejecutar_comando3()
    ejecutar_comando4()
    ejecutar_comando5()
    ejecutar_comando6()
    ejecutar_comando7()
    btn_ejecutar_todos.configure(state="disabled", fg_color="gray")

def matar_todos_los_procesos():
    # Enviar señal SIGINT a todos los procesos ROS 2
    ejecutar_comando("pkill -SIGINT -f ros2", None, None, None, False)
    procesos.clear()
    time.sleep(10)
    subprocess.Popen(["killall", "gnome-terminal-"])
    root.quit()

# Crear botones para cada comando y su botón "Detener" correspondiente
# Inicialmente, los botones "Detener" están deshabilitados y en color gris
btn_comando1 = ctk.CTkButton(root, text="Detección de frutas", command=ejecutar_comando1, fg_color="blue")
btn_comando1.grid(row=0, column=0, pady=5, padx=5, sticky="ew")

btn_comando1d = ctk.CTkButton(root, text="Detener", command=detener_comando1, state="disabled", fg_color="gray")
btn_comando1d.grid(row=0, column=1, pady=5, padx=5, sticky="ew")

btn_comando2 = ctk.CTkButton(root, text="Servicio de picking", command=ejecutar_comando2, fg_color="blue")
btn_comando2.grid(row=1, column=0, pady=5, padx=5, sticky="ew")

btn_comando2d = ctk.CTkButton(root, text="Detener", command=detener_comando2, state="disabled", fg_color="gray")
btn_comando2d.grid(row=1, column=1, pady=5, padx=5, sticky="ew")

btn_comando3 = ctk.CTkButton(root, text="Mensajes de audio", command=ejecutar_comando3, fg_color="blue")
btn_comando3.grid(row=2, column=0, pady=5, padx=5, sticky="ew")

btn_comando3d = ctk.CTkButton(root, text="Detener", command=detener_comando3, state="disabled", fg_color="gray")
btn_comando3d.grid(row=2, column=1, pady=5, padx=5, sticky="ew")

btn_comando4 = ctk.CTkButton(root, text="Procesamiento imágenes", command=ejecutar_comando4, fg_color="blue")
btn_comando4.grid(row=3, column=0, pady=5, padx=5, sticky="ew")

btn_comando4d = ctk.CTkButton(root, text="Detener", command=detener_comando4, state="disabled", fg_color="gray")
btn_comando4d.grid(row=3, column=1, pady=5, padx=5, sticky="ew")

btn_comando5 = ctk.CTkButton(root, text="Nodo central", command=ejecutar_comando5, fg_color="blue")
btn_comando5.grid(row=4, column=0, pady=5, padx=5, sticky="ew")

btn_comando5d = ctk.CTkButton(root, text="Detener", command=detener_comando5, state="disabled", fg_color="gray")
btn_comando5d.grid(row=4, column=1, pady=5, padx=5, sticky="ew")

btn_comando6 = ctk.CTkButton(root, text="Cámara Intel", command=ejecutar_comando6, fg_color="blue")
btn_comando6.grid(row=5, column=0, pady=5, padx=5, sticky="ew")

btn_comando6d = ctk.CTkButton(root, text="Detener", command=detener_comando6, state="disabled", fg_color="gray")
btn_comando6d.grid(row=5, column=1, pady=5, padx=5, sticky="ew")

btn_comando7 = ctk.CTkButton(root, text="Captura de voz", command=ejecutar_comando7, fg_color="blue")
btn_comando7.grid(row=6, column=0, pady=5, padx=5, sticky="ew")

btn_comando7d = ctk.CTkButton(root, text="Detener", command=detener_comando7, state="disabled", fg_color="gray")
btn_comando7d.grid(row=6, column=1, pady=5, padx=5, sticky="ew")

# Botón para ejecutar todos los comandos
btn_ejecutar_todos = ctk.CTkButton(root, text="Ejecutar todos los comandos", command=ejecutar_todos_los_comandos)
btn_ejecutar_todos.grid(row=7, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# Botón rojo para matar todos los procesos y cerrar la interfaz
btn_matar_todos = ctk.CTkButton(root, text="Matar todos los procesos y salir", command=matar_todos_los_procesos, fg_color="red")
btn_matar_todos.grid(row=8, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# Ejecutar el bucle principal de la interfaz
root.mainloop()
