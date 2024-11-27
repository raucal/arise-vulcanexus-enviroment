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
root.title("Interfaz de control CH1 - ARISE")
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
    button_start.configure(state="normal", fg_color="green")
    # Deshabilitar el botón de detener y cambiar su color a gris
    button_stop.configure(state="disabled", fg_color="gray")
    # Eliminar el proceso del diccionario
    if key in procesos:
        procesos.pop(key)

def ejecutar_comando1():
    ejecutar_comando(
        load_ws + " && ros2 run battery_disassembly detect_screws",
        btn_comando1, btn_comando1d, "detect_screws", True)

def detener_comando1():
    detener_comando("pkill -SIGINT -f detect_screws", btn_comando1, btn_comando1d, "detect_screws")

def ejecutar_comando2():
    ejecutar_comando(
        load_ws + " && ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
        btn_comando2, btn_comando2d, "rosbridge_websocket", True)

def detener_comando2():
    detener_comando("pkill -SIGINT -f rosbridge_websocket_launch.xml", btn_comando2, btn_comando2d, "rosbridge_websocket")

def ejecutar_comando3():
    ejecutar_comando(
        load_ws + " && ros2 run battery_disassembly audio_interface",
        btn_comando3, btn_comando3d, "audio_interface", True)

def detener_comando3():
    detener_comando("pkill -SIGINT -f audio_interface", btn_comando3, btn_comando3d, "audio_interface")

def ejecutar_comando4():
    ejecutar_comando(
        load_ws + " && ros2 launch depthai_ros_driver rgbd_pcl.launch.py",
        btn_comando4, btn_comando4d, "rgbd_pcl", True)

def detener_comando4():
    detener_comando("pkill -SIGINT -f rgbd_pcl.launch.py", btn_comando4, btn_comando4d, "rgbd_pcl")

def ejecutar_comando5():
    ejecutar_comando(
        load_ws + " && ros2 run battery_disassembly hri_communication",
        btn_comando5, btn_comando5d, "hri_communication", True)

def detener_comando5():
    detener_comando("pkill -SIGINT -f hri_communication", btn_comando5, btn_comando5d, "hri_communication")

def ejecutar_comando6():
    ejecutar_comando(
        load_ws + " && ros2 run hl2_streamming body_stream",
        btn_comando6, btn_comando6d, "body_stream", True)

def detener_comando6():
    detener_comando("pkill -SIGINT -f body_stream", btn_comando6, btn_comando6d, "body_stream")

def ejecutar_comando7():
    ejecutar_comando(
        "/bin/python3.10 /home/cervera5R/arise-vulcanexus-enviroment/CV2_hololens_stream.py",
        btn_comando7, btn_comando7d, "CV2_hololens_stream", True)

def detener_comando7():
    detener_comando("pkill -SIGINT -f CV2_hololens_stream.py", btn_comando7, btn_comando7d, "CV2_hololens_stream")

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
    print(btn_comando1._state)
    if btn_comando1._state=="disabled": detener_comando1()
    if btn_comando2._state=="disabled": detener_comando2()
    if btn_comando3._state=="disabled": detener_comando3()
    if btn_comando4._state=="disabled": detener_comando4()
    if btn_comando5._state=="disabled": detener_comando5()
    if btn_comando6._state=="disabled": detener_comando6()
    if btn_comando7._state=="disabled": detener_comando7()
    procesos.clear()
    time.sleep(10)
    subprocess.Popen(["killall", "gnome-terminal-"])
    root.quit()

def abrir_enlace():
    url = "http://Cervera5R:Cervera5R47151@192.168.101.91/#Apps"
    subprocess.Popen(['firefox', url])

# Crear botones para cada comando y su botón "Detener" correspondiente
# Inicialmente, los botones "Detener" están deshabilitados y en color gris
btn_comando1 = ctk.CTkButton(root, text="Detección de tornillos", command=ejecutar_comando1, fg_color="green")
btn_comando1.grid(row=0, column=0, pady=5, padx=5, sticky="ew")

btn_comando1d = ctk.CTkButton(root, text="Detener", command=detener_comando1, state="disabled", fg_color="gray")
btn_comando1d.grid(row=0, column=1, pady=5, padx=5, sticky="ew")

btn_comando2 = ctk.CTkButton(root, text="Puente websocket", command=ejecutar_comando2, fg_color="green")
btn_comando2.grid(row=1, column=0, pady=5, padx=5, sticky="ew")

btn_comando2d = ctk.CTkButton(root, text="Detener", command=detener_comando2, state="disabled", fg_color="gray")
btn_comando2d.grid(row=1, column=1, pady=5, padx=5, sticky="ew")

btn_comando3 = ctk.CTkButton(root, text="Mensajes de audio", command=ejecutar_comando3, fg_color="green")
btn_comando3.grid(row=2, column=0, pady=5, padx=5, sticky="ew")

btn_comando3d = ctk.CTkButton(root, text="Detener", command=detener_comando3, state="disabled", fg_color="gray")
btn_comando3d.grid(row=2, column=1, pady=5, padx=5, sticky="ew")

btn_comando4 = ctk.CTkButton(root, text="Cámara Luxonis", command=ejecutar_comando4, fg_color="green")
btn_comando4.grid(row=3, column=0, pady=5, padx=5, sticky="ew")

btn_comando4d = ctk.CTkButton(root, text="Detener", command=detener_comando4, state="disabled", fg_color="gray")
btn_comando4d.grid(row=3, column=1, pady=5, padx=5, sticky="ew")

btn_comando5 = ctk.CTkButton(root, text="Nodo central", command=ejecutar_comando5, fg_color="green")
btn_comando5.grid(row=4, column=0, pady=5, padx=5, sticky="ew")

btn_comando5d = ctk.CTkButton(root, text="Detener", command=detener_comando5, state="disabled", fg_color="gray")
btn_comando5d.grid(row=4, column=1, pady=5, padx=5, sticky="ew")

btn_comando6 = ctk.CTkButton(root, text="Seguimiento de Hololens", command=ejecutar_comando6, fg_color="green")
btn_comando6.grid(row=5, column=0, pady=5, padx=5, sticky="ew")

btn_comando6d = ctk.CTkButton(root, text="Detener", command=detener_comando6, state="disabled", fg_color="gray")
btn_comando6d.grid(row=5, column=1, pady=5, padx=5, sticky="ew")

btn_comando7 = ctk.CTkButton(root, text="Imagen Hololens", command=ejecutar_comando7, fg_color="green")
btn_comando7.grid(row=6, column=0, pady=5, padx=5, sticky="ew")

btn_comando7d = ctk.CTkButton(root, text="Detener", command=detener_comando7, state="disabled", fg_color="gray")
btn_comando7d.grid(row=6, column=1, pady=5, padx=5, sticky="ew")

# Botón para ejecutar todos los comandos
btn_ejecutar_todos = ctk.CTkButton(root, text="Ejecutar todos los comandos", command=ejecutar_todos_los_comandos)
btn_ejecutar_todos.grid(row=7, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# Botón rojo para matar todos los procesos y cerrar la interfaz
btn_matar_todos = ctk.CTkButton(root, text="Matar todos los procesos y salir", command=matar_todos_los_procesos, fg_color="red")
btn_matar_todos.grid(row=8, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# Botón para abrir el enlace en Google Chrome
btn_abrir_enlace = ctk.CTkButton(root, text="Abrir Apps en Firefox", command=abrir_enlace)
btn_abrir_enlace.grid(row=9, column=0, columnspan=2, pady=10, padx=5, sticky="ew")

# Ejecutar el bucle principal de la interfaz
root.mainloop()
