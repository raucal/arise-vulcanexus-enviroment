import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Vector3, Pose
from battery_disassembly import get_mongo_config, claseRobot, claseVentosa, claseLuces, trs, user
from my_robot_interfaces.srv import PickObject, List, CheckInt

from rich import print
import numpy as np
import time
import copy
import traceback
import threading
import multiprocessing

class HRICommunication(Node):
    def __init__(self, config, robot, luces):
        super().__init__('hri_communication')  # Nombre del nodo
        self.config = config
        self.robot = robot
        self.luces = luces

        # BBDD
        self.screws = self.config["screws"] # Posiciones de los tornillos
        self.camera_name = self.config["camera_vision"]["camera_name"]
        self.pose_gripper = self.config["ur5e"]["gripper_pose"]
        if self.camera_name == "intel":
            self.repose_pose = self.config["ur5e"]["intel_repose_pose"]
        elif self.camera_name == "luxonis":
            self.repose_pose = self.config["ur5e"]["luxonis_repose_pose"]
        self.target_pose = self.config["ur5e"]["target_pose"]
        self.speed = self.config["ur5e"]["speed"]
        self.acceleration = self.config["ur5e"]["acceleration"]


        # Variables globales
        self.screws_in_image = None
        self.point_in_rob = None
        self.actual_index = None
        self.robot_in_process = False
        self.screws_not_picked = []
        self.file_path='/home/'+user+'/arise-vulcanexus-enviroment/arise_ws/src/challenge1/battery_disassembly/battery_disassembly/files/lastQRs.txt'
        self.screw_not_reached = None
        self.real_positions = None
        self.head_pose = None
        self.right_hand_pose = None
        self.left_hand_pose = None
        self.T_hol_rob = None
        self.rob_in_hl2 = None
        self.velocity_scale = 1

        # Clientes y servidores
        self.set_publishers_and_subscribers()
        self.set_servers()
    
    def set_publishers_and_subscribers(self):
        self.create_subscription(String, 'battery_disassembly/hololens/qr_coordinates', self.QRCallback, 1) # QR
        self.create_subscription(Vector3, 'battery_disassembly/hololens/finger_position', self.fingerCallback, 10) # Posición del dedo
        self.create_subscription(Int32MultiArray, 'battery_disassembly/camera/screws_in_image', self.screwsCallback, 1 ) # Tornillos presentes en la imagen
        self.create_subscription(String, 'battery_disassembly/hololens/voice_input', self.voiceCallback, 10) # Comandos de voz por parte de Hololens
        
        self.create_subscription(Pose, 'hl2_streamming/head_pose', self.head_callback, 10)
        self.create_subscription(Pose, 'hl2_streamming/left_hand_pose', self.left_hand_callback, 10)
        self.create_subscription(Pose, 'hl2_streamming/right_hand_pose', self.right_hand_callback, 10)
        self.timer = self.create_timer(0.01, self.control_robot)
        
        self.pub_target = self.create_publisher(String, 'battery_disassembly/hololens/screw_info', 2) # Mensaje de success, error o el nº de tornillos iniciales
        self.pub_coords = self.create_publisher(Vector3, 'battery_disassembly/hololens/screws_coordinates', 5) # Mensaje de success, error o el nº de tornillos iniciales
        self.pub_text = self.create_publisher(String, 'audio_interface/play_audio', 2) # Reproducir audio
        self.pub_ur10e = self.create_publisher(String, 'battery_disassembly/robot/send_screw_to_box', 1)

    def set_servers(self):
        # self.create_service(PickObject,'battery_disassembly/robot/pick_screw',self.goToTarget)
        # self.pick_client = self.create_client(PickObject, 'battery_disassembly/robot/pick_screw')

        '''LOS SERVICIOS DE ABAJO YA NO SÉ UTILIZAN PARA COMPROBAR LOS TORNILLOS PRESENTES EN LA IMAGEN'''
        # self.screws_client = self.create_client(List, 'cartif/battery_disassembly/getScrewsList')
        # self.checkScrew_client = self.create_client(CheckInt, 'cartif/battery_disassembly/checkScrewInImage')

    """ ----------------------------------------------------------------
    ROBOT REPOSO
    ---------------------------------------------------------------- """
    def move_robot2repose(self):
        print("Moviendo robot a reposo")
        if self.robot.robot_connected:
            self.robot.moveJ_IK(self.repose_pose, self.speed, self.acceleration)
            self.luces.change_color("VERDE")
        else:
            print("[red]El robot no está conectado")
            self.pub_text.publish(String(data="No he podido moverme a posición de reposo. He perdido la conexión con el equipo."))

    """ ----------------------------------------------------------------
    HOLOLENS: LECTURA QR
    ---------------------------------------------------------------- """ 
    def _request_last_qrs(self, lastQRS=False):
        # Escanear últimos códigos QR
        if lastQRS:
            self.T_hol_rob = self.chargeQRS()
            print("<_request_last_qrs>Esperando a que humano informe sobre Nueva Batería")
        else:
            print("[blue]Escanea los códigos QR con las lentes HL2")
            self.T_hol_rob = None

    def QRCallback(self, data):
        print("[yellow]--------------------------------")
        self.get_logger().info(f'Received QR coordinates: {data.data}')

        if data!=" " and  data.data!="(0.00000, 0.00000, 0.00000);(0.00000, 0.00000, 0.00000);(0.00000, 0.00000, 0.00000)":
            # Escribir las nuevas coordenadas en el txt
            
            with open(self.file_path, 'w') as file:
                file.write(data.data)

            qrs_hl2 = data.data
            self.T_hol_rob = self._build_T_hol_rob(qrs_hl2)

        else:
            print("[red]Información de los códigos QR recibida es inválida")
    
    # Construir la matriz T_hol_rob
    def _build_T_hol_rob(self,qrs_hl2):
        # Parsear la cadena
        qrs_hl2 = (qrs_hl2.replace('(', '').replace(')', '')).split(";")
        qrs=[]
        for qr in qrs_hl2:
            values = qr.split(",")
            vector_np = np.array([float(value) for value in values])
            qrs.append(vector_np)
        q1_hol,q2_hol,q3_hol=qrs[0],qrs[1],qrs[2]

        # Obtener puntos en la referencia del robot
        q1_rob,q2_rob,q3_rob = np.array(self.config["qrs_coordinates"]["RED5R_MESA_ESQUINA_001"]), np.array(self.config["qrs_coordinates"]["RED5R_Referencia_002"]), np.array(self.config["qrs_coordinates"]["RED5R_Referencia_003"]) # origen, arriba izq, abajo der
        
        '''MATRIZ DE TRANSFORMACIÓN HOMOGÉNEA'''
        try:
            # Obtenemos las matrices de transformación homogénea
            T_table_rob=trs.get_transform_matrix_3points(q1_rob,q2_rob,q3_rob)
            T_table_hol=trs.get_transform_matrix_3points(q1_hol,q2_hol,q3_hol)
            T_hol_table=np.linalg.inv(T_table_hol)
            T_hol_rob=np.dot(T_table_rob,T_hol_table)
            self.rob_in_hl2 = trs.refA2refB(np.linalg.inv(T_hol_rob), np.array([0,0,0]))
            print(f"[blue]Robot en referencia Hololens: {self.rob_in_hl2}")

            print("[blue]-----------T_hol_rob ----------- \n: ",T_hol_rob)
            
            return T_hol_rob
        except Exception as e:
            print(f"[red]{e}")
            return None
    
    # Cargar últimas coordenadas QR
    def chargeQRS(self):
        with open(self.file_path, 'r') as file:
            content = file.read()
            if content.strip() == '':
                print(f"[red]El archivo está vacío. Escanea los códigos QR con Hololens")
                return None
            else:
                if content.strip() == "(0.00000, 0.00000, 0.00000);(0.00000, 0.00000, 0.00000);(0.00000, 0.00000, 0.00000)":
                    print("[red]Los valores anteriores son nulos. No se puede construir la matriz de transformación.")
                    return None
                else:
                    return self._build_T_hol_rob(content)

    """ ----------------------------------------------------------------
    HOLOLENS: COORDENADAS TORNILLO EN REF. HOLOLENS
    ---------------------------------------------------------------- """ 
    # Publicar coordenadas 3D de los tornillos para dibujarlos con flechas en la app
    def publish_screws_in_hol(self, screws_indexes):
        for i in screws_indexes:
            if str(i) in self.screws:
                # Transformar de la referencia robot a la referencia de hololens
                screw_in_hol = trs.refA2refB(np.linalg.inv(self.T_hol_rob),np.array([self.screws[str(i)][0][0],self.screws[str(i)][0][1],self.screws[str(i)][0][2],1]))
                self.pub_coords.publish(Vector3(x = screw_in_hol[0], y = screw_in_hol[1], z = screw_in_hol[2]))
    
    """ ----------------------------------------------------------------
    COORDENADAS DEDO/VISTA
    ---------------------------------------------------------------- """ 
    # Recibe las coordenadas del dedo o de la mirada del usuario
    def fingerCallback(self,msg):
        print("[yellow]--------------------------------")
        print(f'[yellow]<fingerCallback> Received Screw coordinates of HL2: {msg}')

        if self.robot.robot_connected and np.all(self.T_hol_rob) != None and self.screws_in_image!=None and not self.robot_in_process and not self.screws_in_image==[] and self.real_positions!= None: 
            # Ponemos bandera tornillo no recogido (Se convierte a true cuando se lo halla)
            self.robot_in_process = True
            # Transformar a la referencia del robot
            self.point_in_rob = list(trs.refA2refB(self.T_hol_rob,np.array([-msg.x,msg.y,msg.z,1]))) # Cambiamos el signo de la coordenada X (HL2 tiene sist. inverso)
            print(f"[blue]<fingerCallback>Punto en la referencia del robot: {self.point_in_rob}")

            # # Llamada al servicio de recogida con el robot
            # req = PickObject.Request()
            # req.point, req.angle = self.point_in_rob, 0.0
            # future = self.pick_client.call_async(req)
            # future.add_done_callback(self._pick_object_response)

        else:
            print("[red]<fingerCallback>Petición rechazada.")
            if not self.robot.robot_connected== None: message = "El robot no está conectado"
            if np.all(self.T_hol_rob) == None: message = "La matriz T_hol_rob no está definida"
            if self.screws_in_image == None: message = "No existen tornillos en la imagen"
            if self.robot_in_process: mea = f"El robot está en proceso de recogida del tornillo {self.actual_index}"
            if self.real_positions== None: message = "No se ha informado de una nueva batería"
            print(f"[red]<fingerCallback>{message}")
            self.pub_text.publish(String(data=f"Petición rechazada. {message}"))
                
    # Procesar respuesta al servicio de recogida con el robot
    def _pick_object_response(self, future):
        try:
            response = future.result()
            if response is not None and response.success:
                print("[green]<_pick_object_response>Movimiento finalizado!")
            else:
                print("[red]<_pick_object_response>Tornillo no recogido")
        except Exception as e:
            print("[red]<_pick_object_response>[ERROR] -> {e}")

    """ ----------------------------------------------------------------
    ROBOT: SERVICIO DE RECOGIDA
    ---------------------------------------------------------------- """ 
    # Maniobras para recoger el tornillo, comprobar si está y llevarlo a la posición destino
    def goToTarget(self, point_in_rob):
        print("[yellow]--------------------------------")
        print(f'[yellow]<goToTarget> Coordenadas Robot recibidas: {point_in_rob}')
        if self.robot.robot_connected:
            try:
                selectedScrew = point_in_rob

                # Encontrar posición del tornillo más cercana a la indicada
                nearest_point = self._find_nearest_point(selectedScrew, self.real_positions)
                index = self._calculate_index(nearest_point)
                print(f"[yellow]<goToTarget>Se ha elegido el tornillo {index} con posicion: {nearest_point}")
                self.actual_index = index
                
                # Tornillos que no han sido recogidos por "Coge el resto"
                # print("Screws not picked: ", self.screws_not_picked)
                
                if not int(index) in self.screws_not_picked:
                     print("<goToTarget>Este tornillo ya ha sido recogido. Aún así se procede a recogerlo.")
                
                # Mover robot
                if int(index)!=4:
                    self.pub_text.publish(String(data="Voy. Recogiendo el tornillo que me has indicado"))
                output, robotOK = self._pickScrew(nearest_point, index)

                print("<goToTarget> output, robotOK: ", output, robotOK)

                if robotOK:
                    print(f"[green]<goToTarget> {output}")
                    self.pub_text.publish(String(data = "He recogido el tornillo"))
                    self.luces.change_color("[[0,0,0],[0,0,0],[0,0,0],[0,0,255]]")
                    success = True
                else:
                    print(f"[red]{output}")
                    success = False # Success indica si el robot ha hecho la maniobra con exito

                # Establecer entorno según lo ocurrido
                self.setEnviroment(robotOK, output) 

            except Exception as e:
                print(f"[red]<goToTarget>[ERROR] -> {e}")
                print(print(traceback.format_exc()))

                success = False
        else:
            if not self.robot.robot_connected: 
                print("[red]<goToTarget> Petición rechazada. Conexión con el robot perdida.")
                self.pub_text.publish(String(data="No he podido procesar la petición. He perdido la conexión con el equipo."))
            success = False
        
        return success

    def _pickScrew(self, coordinates, index):
        print(f"[orange]Recogiendo tornillo...")
        moveOK = False

        try:
            ''' Aproximación'''
            #TODO: Ver porque hacia esto del 3.14
            self.pose_target = coordinates+[0,3.14,0]
            pose_tool = copy.deepcopy(self.pose_gripper)
            pose_tool[2] = -pose_tool[2] - 0.08 # TODO: Cambiar altura cuando funcione de verdad
            self.pose_approach = self.robot.poseTrans(self.pose_target,pose_tool)
            moveOK = self.robot.moveJ_IK(self.pose_approach, self.speed, self.acceleration)

            ''' Coger tornillo '''
            moveOK = self.robot.moveL(self.pose_target, self.speed, self.acceleration)

            '''Activar magnetismo'''
            self.robot.setToolDigitalOut(1, True)

            time.sleep(0.5)

            ''' Sacar tornillo '''
            moveOK = self.robot.moveL(self.pose_approach, self.speed, self.acceleration)

            ''' Comprobación '''
            if moveOK:

                ''' Mover a zona de reposo '''
                self.move_robot2repose()

                print("Comprobando si se ha recogido tornillo")
                self.pub_text.publish(String(data="Comprobando si he recogido el tornillo"))
                time.sleep(3) # Por si los frames de la cámara van con retraso
                detected = self._checkScrew(int(index))
                print("<_pickScrew> ¿Detectado? ", detected)
                if not detected:
                    return "Tornillo recogido", True
                else:
                    return "Tornillo no recogido", False
            else:
                return "Robot no alcanza posición", False

        except Exception as e:
            print(f"[red]{e}")
            return "El robot ha fallado en su ejecución", False

    def setEnviroment(self, robotOK, output, audio = True):
        print("<setEnviroment>Estableciendo el entorno...")

        # TORNILLO RECOGIDO: Si se ha recogido el tornillo se lleva al UR10e
        if robotOK: 
            # TODO: Comprobar si esto lo tengo que enviar aquí o despues de llevar el tornillo al ur10e
            self.pub_target.publish(String(data = "success")) # Informar a Hololens

            # HACK: Ya no se lleva al UR10, se deja en una caja
            # ''' Llevar al UR10e y volver a reposo '''
            # self.goToUR10e(audio)
            self.placeInBox()

            ''' Eliminar tornillo '''
            try:
                self.screws_not_picked.remove(int(self.actual_index))
            except:
                pass

            ''' Informar '''
            self._inform_about_screws_not_picked()

        # TORNILLO NO RECOGIDO: Si no se ha recogido el tornillo se aproxima el robot al tornillo para que el usuario lo coja con la garra
        else:
            self.pub_target.publish(String(data = "error")) # Informar a Hololens
            self.luces.change_color("[[0,0,0],[0,0,0],[0,0,0],[255,128,0]]")

            if output=="Robot no alcanza posición":
                print(f"[red]<setEnviroment>Robot no alcanza el tonrillo {self.actual_index}")
                self.robot_in_process=False
                self.pub_text.publish(String(data="No alcanzo el tornillo que me has indicado. Recógelo tú"))
                self.screw_not_reached = True
            else:
                time.sleep(1)
                print("Aproximando a tornillo...")
                self.pub_text.publish(String(data="No he podido recoger el tornillo, recógelo tú con mi garra."))

                ''' Aproximación '''
                self.robot.moveJ_IK(self.pose_approach, self.speed, self.acceleration)

                ''' Acercar '''
                pose_toolFree = [0,0,-0.02,0,0,0]
                pose_free=self.robot.poseTrans(self.pose_target,pose_toolFree)
                self.robot.moveL(pose_free)

                ''' Activar freedrive'''
                # rc.freedriveMode([0,0,1,0,0,1])
                self.robot.freedriveMode([0,0,1,0,0,1],self.robot.getForwardKinematics()) 
                print("Freedrive activado")
                self.pub_text.publish(String(data="Freedrive activado"))
    
    def goToUR10e(self, audio = True):
        print("Llevando tornillo a Odín...")
        if audio:
            self.pub_text.publish(String(data="LLevando el tornillo a destino"))

        ''' Aproximación '''
        pose2 = self.robot.poseTrans(self.target_pose, [0,0,-0.10,0,0,0])
        self.robot.moveJ_IK(pose2, self.speed, self.acceleration)
        
        ''' Dejar tornillo '''
        self.robot.moveL(self.target_pose, self.speed, self.acceleration)

        ''' Enviar señal al ur10e'''
        self.pub_ur10e.publish(String(data="Send To Box"))
        time.sleep(1)

        ''' Alejar '''
        self.pub_text.publish(String(data="Volviendo a la posición de reposo"))
        pose3 = self.robot.poseTrans(self.robot.getForwardKinematics(),[0,0,-0.05,0,0,0])
        self.robot.moveL(pose3, self.speed, self.acceleration)

        '''Reposo'''
        self.move_robot2repose()

        self.robot_in_process = False
    
    def placeInBox(self, audio = True):
        print("Dejando tornillo en la caja")
        if audio:
            self.pub_text.publish(String(data="Dejando tornillo en la caja"))
        
        ''' Dejar tornillo '''
        pose_aprox = self.robot.poseTrans(self.target_pose, [0,0,-0.2,0,0,0])
        self.robot.moveJ_IK(pose_aprox, self.speed, self.acceleration)
        self.robot.moveL(self.target_pose, self.speed, self.acceleration)

        '''Soltar tornillo'''
        self.robot.setToolDigitalOut(1, False)
        self.robot.setToolDigitalOut(0, True)
        time.sleep(2)
        self.robot.setToolDigitalOut(0, False)

        ''' Alejar '''
        self.pub_text.publish(String(data="Volviendo a la posición de reposo"))
        pose3 = self.robot.poseTrans(self.robot.getForwardKinematics(),[0,0,-0.05,0,0,0])
        self.robot.moveL(pose3, self.speed, self.acceleration)

        '''Reposo'''
        self.move_robot2repose()

        self.robot_in_process = False



    """ ----------------------------------------------------------------
    HOLOLENS: COMANDOS DE VOZ
    ---------------------------------------------------------------- """ 
    def sendScrewsInfo2Hololens(self):
        print("<sendScrewsInfo2Hololens>Numero de tornillos ",len(self.initial_indexes))
        self.pub_target.publish(String(data=str(len(self.initial_indexes))))

        # '''PUBLICAR COORDENADAS EN HOLOLENS'''
        print("Publicando coordenadas de los tornillos")
        self.publish_screws_in_hol(self.initial_indexes)
        
    # Comandos de voz de Hololens
    def voiceCallback(self, data):
        if np.all(self.T_hol_rob) != None and self.robot.robot_connected==True: 
            if data.data == "Nueva bater?a":
                    print("[orange]<voiceCallback>Echo: Nueva batería")
                    self.screws_not_picked.clear()
                    try:
                        self._getScrewsInfo() # Resetear parámetros
                        self.sendScrewsInfo2Hololens() # Informar a Hololens
                    except Exception as e:
                        print(print(f"[red]ERROR -> {e}"))
                        self.pub_text.publish(String(data="Error. Echa un ojo al programa"))
            else:
                # Solo se puede llevar el tornillo cuando se ha recogido
                if data.data == "Tornillo recogido":
                    if self.robot_in_process:
                        print(f"[orange]<voiceCallback>Echo: Tornillo recogido. LLevando tornillo {self.actual_index} a Odín")
                        self.pub_text.publish(String(data="De acuerdo. LLevando el tornillo a Odín"))

                        ''' Deshabilitar freedrive'''
                        self.robot.endFreedriveMode() # Desactivamos Freedrive

                        ''' Alejar '''
                        pose_aux = [0,0,-0.055,0,0,0] 
                        pose1 = self.robot.poseTrans(self.robot.getForwardKinematics(),pose_aux)
                        self.robot.moveL(pose1, self.speed, self.acceleration)

                        # Mover tornillo al UR10e, establecer entorno de recogida
                        self.setEnviroment(True,"Tornillo recogido", audio = False)

                    else:
                        if self.screw_not_reached:
                            print(f"[orange]<voiceCallback>Echo: Tornillo recogido. Humano a recogido el tornillo {self.actual_index} manualmente")
                            self.pub_target.publish(String(data = "success")) # Informar a Hololens
                            self.screw_not_reached = None
                            self.screws_not_picked.remove(int(self.actual_index))
                            self._inform_about_screws_not_picked()
                            self.luces.change_color("VERDE")
                        else:
                            if self.actual_index == None: print(f"[red]<voiceCallback>Petición rechazada. No se puede recoger un tornillo que aún no se ha seleccionado")
                            if not self.robot_in_process: print(f"[red]<voiceCallback>Petición rechazada. El robot no está en ningún proceso de recogida.")

                elif data.data == "Coge el resto":
                    print("[orange]<voiceCallback>Echo: Coge el resto")
                    self.pub_text.publish(String(data="Voy. Recogiendo todos los tornillos"))

                    # Recorremos los índices de los tornillos que quedan por recoger y enviamos su posición a _pickScrews().
                    # Aquí si no lo coge, no vuelve a por ello, avisa al usuario al final de los que no ha podido recoger
                    for i in self.screws_not_picked:
                        output, robotOK = self._pickScrew(self.screws[i][0], i) 

                        if robotOK:
                            print(f"[green]<voiceCallback>{output}")
                            self.pub_target.publish(String(data = "success")) # Informar a Hololens
                            self.pub_text.publish(String(data = "He recogido el tornillo"))
                            self.luces.change_color("[[0,0,0],[0,0,0],[0,0,0],[0,0,255]]")
                            self.screws_not_picked.remove(int(self.actual_index))
                            # Llevar tornillo al UR10e
                            self.goToUR10e()
                        else:
                            print(f"[red]<voiceCallback>{output}")
                            self.pub_target.publish(String(data = "error")) # Informar a Hololens
                            self.pub_text.publish(String(data = f"No he podido recoger el tornillo. Motivo: {output}"))
                            self.luces.change_color("[[0,0,0],[0,0,0],[0,0,0],[255,128,0]]")

        else:
            print("[red]<voiceCallback>Petición rechazada")
            if np.all(self.T_hol_rob) == None: print("[red]<voiceCallback>La matriz T_hol_rob no está definida")
            if not self.robot.robot_connected: 
                print("[red]<voiceCallback>El robot no está conectado")
                self.pub_text.publish(String(data="No he podido procesar la petición. He perdido la conexión con el equipo."))
            
    """ ----------------------------------------------------------------
    TORNILLOS
    ---------------------------------------------------------------- """ 
    # Callback de los tornillos presentes en la imagen en cada frame
    def screwsCallback(self, array):
        # print(array)
        self.screws_in_image = list(array.data)
        # print("Tornillos presentes: ", self.screws_in_image)
    
    # Guardar los tornillos iniciales y sus posiciones correspondientes
    def _getScrewsInfo(self):
        # Cargar tornillos de la imagen
        print("<_getScrewsInfo> Cargando tornillos...")
        
        '''YA NO SE UTILIZA -> Cuando obtenemos los tornillos por servicio'''
        # req = List.Request()
        # self.future = self.screws_client.call_async(req)
        # rclpy.spin_until_future_complete(self, self.future)
        # self.initial_indexes = self.future.result().screws

        '''Cuando obtenemos los tornillos por suscripción'''
        self.initial_indexes = self.screws_in_image
        self.real_positions = self._getScrewsPositions()
        for i in self.initial_indexes:
            self.screws_not_picked.append(int(i)) # Al principio los tornillos no recogidos son todos

        # Informar por consola y por audio
        print("[yello]<_getScrewsInfo>Tornillos iniciales: ",self.initial_indexes)
        self.pub_text.publish(String(data="Se tienen que recoger "+str(len(self.initial_indexes))+" tornillos")) 

    # Comprobar si un índice de un tornillo está presente en la imagen comparandolo con los mensajes del topic
    def _checkScrew(self, index):
        print(f"Comprobando si el tornillo {index} está en la imagen...")
        print("Tornillos en imagen: ", self.screws_in_image)
        if int(index) in self.screws_in_image:
            print("[orange3]<_checkScrew> Tornillo presente en la imagen")
            return True
        else:
            print("[green_yellow]<_checkScrew> Tornillo no presente en la imagen")
            return False

    # YA NO SÉ UTILIZA ESTE SERVICIO
    '''def _check_screw_response(self, req):
        future = self.checkScrew_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            print("Response: ", response)
            if response.is_present: self.detected = True
            else: self.detected = False
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')'''

    # Encontrar la posición real del tornillo más cercana a la capturada con Hololens 2 (referenciada al robot)
    def _find_nearest_point(self, measure_position, real_positions):
        # print(f"Comparando {measure_position} con {real_positions}")
        distances=[]
        for real_pos in real_positions:
            distance = trs.euclidean_distance3D(np.array(real_pos), np.array(measure_position))
            distances.append(distance)

        min_distance = min(distances)
        nearest_point_index = distances.index(min_distance)
        nearest_point = real_positions[nearest_point_index]
        return nearest_point
    
    # Comprobar que índice corresponde a la posición 3D (robot) del tornillo y devolver su índice
    def _calculate_index(self, point):
        for clave, valor in self.screws.items():
            if valor[0]==point:
                index=clave
                return index

    # Obtener las posiciones de cada tornillo presente en la imagen
    def _getScrewsPositions(self):
        real_positions = []
        # print("Initial screws: ",self.initial_indexes)
        for screw in self.initial_indexes:
            if str(screw) in self.screws:
                real_positions.append(self.screws[str(screw)][0])
        # print(f"[yellow]Posiciones: {real_positions}")
        return real_positions
            
    def _inform_about_screws_not_picked(self):
        print(f"[yellow]Quedan {str(len(self.screws_not_picked))} tornillos por recoger -> {self.screws_not_picked}")
        if str(len(self.screws_not_picked))=="1": self.pub_text.publish(String(data="Queda un tornillo por recoger"))
        elif str(len(self.screws_not_picked))=="0": self.pub_text.publish(String(data="Se han recogido todos los tornillos"))
        else: self.pub_text.publish(String(data="Quedan "+str(len(self.screws_not_picked))+" tornillos por recoger"))
        
    """ ----------------------------------------------------------------
    HOLOLENS
    ---------------------------------------------------------------- """ 
    def head_callback(self, msg):
        self.head_pose = np.array([-msg.position.x, msg.position.y, -msg.position.z])
    
    def right_hand_callback(self, msg):
        self.right_hand_pose = np.array([-msg.position.x, msg.position.y, -msg.position.z])
    
    def left_hand_callback(self, msg):
        self.left_hand_pose = np.array([-msg.position.x, msg.position.y, -msg.position.z])
    
    def control_robot(self):
        if np.all(self.head_pose) != None and np.all(self.rob_in_hl2) !=None:
            dist_head = trs.euclidean_distance3D(self.head_pose, self.rob_in_hl2)
            
            if dist_head <= self.robot.range_radius:
                current_scale = 0.1
            elif dist_head > (self.robot.range_radius) and dist_head <= (self.robot.range_radius+0.1):
                current_scale= 0.3
            elif dist_head > (self.robot.range_radius+0.1) and dist_head <= (self.robot.range_radius+0.2):
                current_scale = 0.6
            elif dist_head > (self.robot.range_radius+0.2) and dist_head <= (self.robot.range_radius+0.3):
                current_scale = 0.8
            else:
                current_scale = 1
            
            # print(f"[orange1]Distancias --> HEAD: {dist_head}")
            # print("Current scale: ", current_scale)
            if self.velocity_scale!=current_scale:
                self.velocity_scale = current_scale
                print("Escala: ", self.velocity_scale)
                self.robot.setSpeedSlider(self.velocity_scale)
                if self.velocity_scale<0.6 and self.robot_in_process:
                    self.pub_text.publish(String(data = "Aléjate de mí"))

# Hilo para recibir independientemente los callbacks
def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        config = get_mongo_config()
        luces = claseLuces.TableLights(config["ip_lights"])
        robot = claseRobot.Robot(
            ip=config["ur5e"]["ip_address"],
            mode=config["ur5e"]["mode"],
            range_radius=config["ur5e"]["range_radius"],
            luces=luces,
            autoreconnection=True,
            io_connection=True
        )
        
        if robot.robot_connected:
            # Inicializar valores de las salidas de la herramienta
            robot.setToolDigitalOut(0, False)
            robot.setToolDigitalOut(1, False)
            hri_node = HRICommunication(config, robot, luces)
            if robot.mode == 'control':
                hri_node.move_robot2repose()
            hri_node._request_last_qrs(lastQRS=True)

            # Iniciar el hilo para el spin del nodo
            thread = threading.Thread(target=spin_node, args=(hri_node,))
            thread.start()
            
            try:
                while rclpy.ok() and robot.robot_connected:
                    if hri_node.point_in_rob!=None:
                        point = hri_node.point_in_rob
                        hri_node.point_in_rob = None    
                        success = hri_node.goToTarget(point)
                        if success:
                            luces.change_color("VERDE")
            except KeyboardInterrupt:
                print("Interrupción detectada. Cerrando el programa...")
    except Exception as e:
        print(f"[red]ERROR -> {e}")
    finally:
        try:
            if 'hri_node' in locals():
                hri_node.destroy_node()
            luces.gateway_disconnection()
            print("Cerrando programa")
            rclpy.shutdown()
            if 'thread' in locals():
                thread.join()
        except Exception as e:
            print(f"[red]ERROR al cerrar -> {e}")

if __name__ == '__main__':
    main()
