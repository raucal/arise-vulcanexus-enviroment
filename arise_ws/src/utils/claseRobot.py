import rtde_receive
import rtde_control
import rtde_io
from rich import print
import socket
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import threading
import logging
import subprocess
import signal
import sys
import multiprocessing

class Robot():
    def __init__(self, ip, mode, range_radius = None, luces=None, show=None, autoreconnection = False, io_connection = False):
        self.ip= ip
        self.mode= mode
        self.range_radius = range_radius
        self.luces = luces
        self.show = show
        self.autoreconnection = autoreconnection
        self.io_connection = io_connection

        self.check_connections = False
        self.robot_connected = False
        self.repose_pose = None
        self.robot_connected = False
        self.status = 0  # Inicializamos status como una variable normal
        self.robot_connected = False
        self.protective_stop = None
        self.robot_emergency = None
        self.remote_connected = None
        self._running = False

        self.robot_connected, self.error_robot = self.rtde_connection()
        if self.robot_connected:
            if self.io_connection:
                self.io_connected, _ = self.rtde_io_connection()
            
            self.thread = threading.Thread(target=self.check_connection)
            self._running = True
            self.thread.start()
        
        signal.signal(signal.SIGINT, self.handle_interrupt)

        
    def rtde_connection(self):
        # print(f"<claseRobot> Comprobando si {self.ip} es accesible")
        robot_in_net, _ = self.ping_ip(self.ip)
        if robot_in_net:
            # print("[green]<claseRobot>Dirección accesible!")
            print(f"[white]<claseRobot> Estableciendo conexión con {self.ip} en modo {self.mode}")
            try:  
                self.rr=rtde_receive.RTDEReceiveInterface(self.ip)
                print("[green]<claseRobot> -> Conexión en modo Receive establecida")

                if self.mode=='control':
                    try:
                        self.rc=rtde_control.RTDEControlInterface(self.ip)
                        if self.rc!=None:
                            self.check_connections=True
                            if self.luces!=None:
                                self.luces.change_color("VERDE")
                            print("[green]<claseRobot> -> Conexión en modo Control establecida")
                            return True, None
                        else:
                            return False, None
                    except Exception as e:
                        print(f"[red]>>[ERROR]: {e}")
                        self.rc = None
                        return False, str(e)
                else:
                    self.check_connections=True
                    return True, None
                
                
            except Exception as e:
                print(f"[red>]<claseRobot> No se ha establecido conexión con {self.ip}")
                print(f"[red]->[ERROR]: {e}")
                self.rr = None
                return False, str(e)
        else:
            print("[red]<claseRobot> HOST inaccesible. No se ha podido establecer conexión con el robot.")
            return False, "HOST inaccesible"

    def rtde_disconnection(self):
        if self.robot_connected: 
            self.receive_disconnection()
            if self.mode == 'control': self.control_disconnection()
            if self.io_connection: self.io_disconnection()

    def ping_ip(self, ip_address):
        try:
            # Ejecutar el comando ping con timeout de 2 segundos
            result = subprocess.run(['ping', '-c', '4', '-W', '2', ip_address], capture_output=True, text=True, timeout=5)

            # Verificar el código de salida
            if result.returncode == 0:
                # El ping fue exitoso
                print(f"[green]La dirección IP {ip_address} es accesible.")
                return True, result.stdout
            else:
                # El ping no fue exitoso
                error_message = result.stderr.strip() if result.stderr else "Error al hacer ping"
                print(f"Error al hacer ping: {error_message}")
                return False, error_message

        except subprocess.TimeoutExpired:
            # El comando ping excedió el tiempo de espera
            print(f"<claseRobot> Tiempo de espera excedido al hacer ping a {ip_address}.")
            return False, f"Tiempo de espera excedido al hacer ping a {ip_address}."
        except Exception as e:
            # Otra excepción ocurrió
            print(f"<claseRobot> Error al ejecutar el comando ping: {e}")
            return False, "Error al ejecutar el comando ping."
    
    def set_repose_pose(self, pose):
        self.set_repose_pose = pose

    """ ----------------------------------------------------------------
    CONTROL
    ---------------------------------------------------------------- """ 
    def control_connection(self):
        print(f"Comprobando si {self.ip} es accesible")
        robot_in_net, _ = self.ping_ip(self.ip)
        if robot_in_net:
            try:  
                if self.mode=='control':
                    self.rc=rtde_control.RTDEControlInterface(self.ip)
                    print(f"[green]Conexión control establecida {self.ip}")
                return True, "Robot conectado"
            except Exception as e:
                print(f"[red]Conexión no establecida con {self.ip}")
                print(f"[red]->[ERROR]: {e}")
                return False, str(e)
        else:
            print("[red]HOST inaccesible. No se ha podido establecer conexión con el robot.")
            return False, "HOST inaccesible"
    
    def control_disconnection(self):
        if self.robot_connected:
            self.rc.disconnect()
            self.rc = None
            print("[green]Control disconnected")

    def control_reconnect(self):
        try:  
            self.rc.reconnect()
            print("[green]Control reconnection established with ",self.ip)
            return True, "Robot reconectado"
        except Exception as e:
            print("[red]Control reconnection not established with ",self.ip)
            print("[red]->ERROR: ",e)
            return False, str(e)

    def isControlConnected(self):
        return self.rc.isConnected()
    
    def isProgramRunning(self):
        return self.rc.isProgramRunning()

    def control_stopScript(self):
        self.rc.stopScript()
    
    def control_reuploadScript(self):
        self.rc.reuploadScript()

    def isProgramRunning(self):
        return self.rc.isProgramRunning()
    
    def getForwardKinematics(self):
        return self.rc.getForwardKinematics()
    
    def getInverseKinematicsHasSolution(self,pose):
        return self.rc.getInverseKinematicsHasSolution(pose)

    def poseTrans(self,frame,pose_in_frame):
        try: 
            return self.rc.poseTrans(frame,pose_in_frame)
        except Exception as e:
            return None

    def MakeToolSpeedZ(self):
        dir = self.rr.getActualTCPPose()
        dir = [0,0,0,dir[3],dir[4],dir[5]]
        sp =self.rc.poseTrans(dir, [0,0,1,0,0,0])
        pose_trans = [sp[0], sp[1], sp[2], 0, 0, 0]
        return pose_trans

    def moveUntilContact(self,pose,acc=0.2):
        if self.luces!=None:
            self.luces.change_color("ROJO")
        contact = self.rc.moveUntilContact(pose,acceleration=acc)
        if self.luces!=None:
            self.luces.change_color("VERDE")
        return contact
    
    def freedriveMode(self, free_axes=[1,1,1,1,1,1], feature = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        if self.luces!=None:
            self.luces.change_color("AMARILLO")
        self.rc.freedriveMode(free_axes, feature)
    
    def endFreedriveMode(self):
        self.rc.endFreedriveMode()
        if self.luces!=None:
            self.luces.change_color("VERDE")
      
    def moveJ_IK(self,pose,speed=0.2,acceleration=0.2):
            try:
                if self.getInverseKinematicsHasSolution(pose):
                    if self.show:
                        print(f"[orange3]Moviendo a {pose}")
                    if self.luces!=None:
                        self.luces.change_color("ROJO")
                    self.rc.moveJ_IK(pose,speed,acceleration)
                    # Suele dar problemas al hacer muchos moveJ seguidos -> Poner un time.sleep o no poner luces en verde
                    # if self.luces!=None:
                    #     self.luces.change_color("VERDE")
                    return True
                else:
                    print(f"[red]El robot no alcanza la posición {pose}")
                    return False
            except Exception as e:
                print(f"[red]->[ERROR]: {e}")

    def moveL(self,pose, speed=0.2,acceleration=0.2):
            try:
                if self.getInverseKinematicsHasSolution(pose):
                    if self.show:
                        print(f"[orange3]Moviendo a {pose}")
                    if self.luces!=None:
                        self.luces.change_color("ROJO")
                    self.rc.moveL(pose,speed,acceleration)
                    # if self.luces!=None:
                    #     self.luces.change_color("VERDE")
                    return True
                else:
                    print(f"[red]El robot no alcanza la posición {pose}")
                    return False
            except:
                return False

    def pickCardboard(self, vgp20, cardboard_coord, distance = 0.3, vacuum = True):
        pose_actual = self.getActualTCPPose()
        if not list(cardboard_coord)==self.conveyor[:3]:
            # (0) Guardar posición actual
            pose_object=list(cardboard_coord)+pose_actual[-3:]
        else:
            pose_object=self.conveyor

        # (1) Aproximar
        pose_approach=self.poseTrans(pose_object, [0,0,-distance,0,0,0])
        if self.luces!=None:
            self.luces.change_color("ROJO")
        self.moveJ_IK(pose_approach)

        # (2) Mover hasta contacto
        pose_trans=self.MakeToolSpeedZ()
        contact = self.moveUntilContact(pose_trans)

        if contact:
            # (3) Presionar el objeto
            pressure_pose=[0,0,0.02,0,0,0]
            pose_vacuum=self.poseTrans(self.getForwardKinematics(),pressure_pose)
            self.moveL(pose_vacuum,0.1,0.1)

            # (4.a) Absorber/Soltar
            if vacuum:
                vgp20.vacuum_on()
            else:
                vgp20.release_vacuum()
            
            # (5) Alejar
            self.moveL(pose_approach)

            # (6) Volver a pose inicial
            print("Vuelta a la pose inicial")
            self.moveJ_IK(pose_actual)
        if self.luces!=None:
            self.luces.change_color("ROJO")

        configuration = self.getActualQ()
        configuration[-1]=0
        self.moveJ(configuration)

        if self.luces!=None:
            self.luces.change_color("VERDE")

    def moveJ(self,pose,speed=0.2,acceleration=0.2):
        '''Falta hacer la comprobación de cinemática inversa para la configuración'''
        if self.show:
            print("Moviendo a ",pose)
        try:
            if self.luces!=None:
                self.luces.change_color("ROJO")
            self.rc.moveJ(pose,speed,acceleration)
            # if self.luces!=None:
            #     self.luces.change_color("VERDE")
        except:
            ...
    
    """ ----------------------------------------------------------------
    HILO CONEXIÓN
    ---------------------------------------------------------------- """ 
    
    def terminate(self):
        # Finalizamos el hilo
        print("<claseRobot> Finalizando el hilo...")
        self._running=False
        self.thread.join()
    
    def handle_interrupt(self, signum, frame):
        # Manejar Ctrl+C
        print("\n<claseRobot> Se recibió Ctrl+C. Finalizando el hilo...")

        self.check_connections=False
        self.rtde_disconnection()
        self.robot_connected = False

        if self.thread.is_alive():
            self.terminate()
            

    def check_connection(self):
        try:
            while self._running:
                if self.check_connections:
                    
                    self.protective_stop = self.isProtectiveStopped()
                    self.robot_emergency = self.isEmergencyStopped()
                    if self.mode == 'control':  
                        self.remote_connected = self.open_send_close_dashboard("is in remote control")[48:53]

                    # print(self.remote_connected[48:53])
                    # print(self.remote_connected , self.protective_stop, self.robot_emergency)

                    if self.robot_emergency or self.protective_stop or self.remote_connected=="false":
                        self.check_connections=False
                        if self.luces!=None:
                            self.luces.change_color("ROJO")
                        if self.robot_emergency: print("[red]Parada de emergencia activada. Levante la seta de emergencia, coloque el robot manualmente y reconecte")
                        if self.protective_stop: print("[red]Parada de protección activada.")
                        if self.remote_connected: print(f"[red]Robot desconectado de modo remoto")
                        
                        self.rtde_disconnection()
                        self.robot_connected=False
                        
                        if self.autoreconnection:
                            retry=0
                            while not self.robot_connected and retry<3:
                                print("[orange1]<claseRobot> Autoreconectando en 20 segundos...")
                                retry+=0
                                time.sleep(20)
                                self.robot_connected, self.error_robot=self.rtde_connection()
                                if not self.robot_connected:
                                    print("[red]No se ha podido establecer la conexión")
                                    self.rtde_disconnection()

                            if self.robot_connected and self.repose_pose!=None:
                                retry=0
                                print("<claseRobot> Volviendo a posición de reposo")
                                self.moveJ_IK(self.repose_pose)
                        else:
                            print("Habilite el robot, coloquelo manualmente y reconecte")
        except KeyboardInterrupt:
            ...

    """ ----------------------------------------------------------------
    RECEIVE
    ---------------------------------------------------------------- """ 
    def receive_reconnect(self):
        try:  
            self.rr.reconnect()
            print("[green]Receive reconnection established with ",self.ip)
            return True
        except Exception as e:
            print("[red]Receive reconnection not established with ",self.ip)
            print("[red]->ERROR: ",e)
            return False

    def receive_disconnection(self):
        if self.robot_connected:
            self.rr.disconnect()
            self.rr = None
            print("[green]Receive disconnected")
        
    def getRobotMode(self):
        return self.rr.getRobotMode()
    def getRobotStatus(self):
        return self.rr.getRobotStatus()
    def isEmergencyStopped(self):
        return self.rr.isEmergencyStopped()
    def getActualQd(self):
        return self.rr.getActualQd()
    def getActualQ(self):
        return self.rr.getActualQ()
    def getActualTCPPose(self):
        return self.rr.getActualTCPPose()
    def isProtectiveStopped(self):
        return self.rr.isProtectiveStopped()
    def isReceiveConnected(self):
        return self.rr.isConnected()

    """ ----------------------------------------------------------------
    IO
    ---------------------------------------------------------------- """ 
    def rtde_io_connection(self):
        print(f"[white]<claseRobot> Estableciendo conexión con entradas y salidas")
        try:
            self.ri = rtde_io.RTDEIOInterface(self.ip)
            print("[green]<claseRobot> -> Conexión en modo IO establecida")
            return True, None
        except Exception as e:
            print(f"[red>]No se ha establecido conexión con entradas y salidas")
            print(f"[red]->[ERROR]: {e}")
            return False, e
    
    def setToolDigitalOut(self, number, value):
            print(f"[green]Activando salida digital de la herramienta {number} con valor {value}")
            return self.ri.setToolDigitalOut(number, value)
    
    def setSpeedSlider(self, value):
        try:
            self.ri.setSpeedSlider(value)
        except:
            pass
    
    def io_disconnection(self):
        try:
            self.ri.disconnect()
            print("[green]IO disconnected")
        except:
            pass

    """ ----------------------------------------------------------------
    DASHBOARD
    ---------------------------------------------------------------- """ 
    def dashboard_connection(self):
        self.dash_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dash_client.settimeout(2)
        self.dash_client.connect((self.ip, 29999))
    
    def send_dashboard_command(self, command):
        cmd = (command + "\n")
        self.dash_client.sendall(cmd.encode())
        # time.sleep(5)
        rcvd = self.dash_client.recv(4096)
        return str(rcvd)
    
    def open_send_close_dashboard(self,command):
        self.dash_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dash_client.settimeout(2)
        self.dash_client.connect((self.ip, 29999))
        cmd = (command + "\n")
        self.dash_client.sendall(cmd.encode())
        time.sleep(2)
        rcvd = self.dash_client.recv(4096)
        self.dash_client.close()
        return str(rcvd)
    


    


# robot = Robot(ip = "172.17.0.2", mode = "receive", autoreconnection = True, io_connection=True)
