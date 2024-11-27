import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import PickObject, TransformCoordinates
from example_interfaces.srv import Trigger
from fruit_picking import get_mongo_config, claseRobot, clasePinza, claseLuces, trs
from std_msgs.msg import String
from rich import print
import numpy as np
import traceback

class PickRobot(Node):
    def __init__(self, config, robot=None, gripper=None, luces = None):
        super().__init__('pick_node')
        self.config = config
        self.robot = robot
        self.gripper = gripper
        self.luces = luces
        
        self.repose_pose = self.config["ur10e"]["repose_pose"]
        self.target_pose = self.config["ur10e"]["target_pose"]
        self.gripper_pose = self.config["ur10e"]["gripper_pose"]
        self.security_plan = self.config["ur10e"]["security_plan"]
        self.T_cam_rob = config["ur10e"]["T_cam_tcp"]
        self.speed = self.config["ur10e"]["speed"]
        self.acceleration = self.config["ur10e"]["acceleration"]

        # Iniciar comunicadores
        self.set_publishers_and_subscribers()
        self.set_servers()

    def set_publishers_and_subscribers(self):
        self.pub_text = self.create_publisher(String, 'audio_interface/play_audio', 2) # Reproducir audio

    def set_servers(self):
        self.create_service(PickObject, 'fruit_picking/robot/pick_object', self.pick_fruit)
        self.create_service(Trigger, 'fruit_picking/robot/place_object', self.place_fruit)
        self.create_service(Trigger, 'fruit_picking/robot/move2repose', self.move_robot2repose)
        self.create_service(TransformCoordinates, 'fruit_picking/robot/camera2robot', self.cam2robot)
        self.create_service(Trigger, 'fruit_picking/robot/reconnect', self.reconnect)
    
    """ ----------------------------------------------------------------
    SERVICIO: Mover robot a reposo
    ---------------------------------------------------------------- """ 
    def move_robot2repose(self, request, response):
        print("[yellow]----------------------------------")
        print("<move_robot2repose>Moviendo  robot a reposo...")
        self.pub_text.publish(String(data="Moviéndome a posición de reposo"))

        if self.robot.robot_connected:
            try:
                moveOK = self.robot.moveJ_IK(self.repose_pose, self.speed, self.acceleration)
                if moveOK:
                    self.luces.change_color("VERDE")
                    response.success = True
                    response.message = ""
                else:
                    response.success = False
                    response.message = "Robot no se ha movido a posición de reposo"
            except Exception as e:
                response.success = False
                response.message = e
        else:
            print("[red]El robot no está conectado")
            response.success = False
            response.message = "El robot no está conectado"
        
        if not response.success:
            self.pub_text.publish(String(data="No he podido moverme a posición de reposo. He perdido la conexión con el equipo."))
        
        return response

    """ ----------------------------------------------------------------
    SERVICIO: Recoger objeto con pinza robotiq
    ---------------------------------------------------------------- """ 
    def pick_fruit(self, request, response):
        print("[blue]--------------- PICK -------------------")
        point = [request.point.x, request.point.y, request.point.z]
        # Comprobación de la altura
        if point[2]<=self.security_plan:
            print("[red]<pick_fruit>Petición rechazada. Posición detectada del objeto traspasa el plano de seguridad")
            self.pub_text.publish(String(data="Petición rechazada. Posición detectada del objeto traspasa el plano de seguridad"))
            response.success = False
        else:
            print(f"[yellow]<pick_fruit>Recogiendo fruta en coordenadas: {request.point} con ángulo: {request.angle}")
            self.pub_text.publish(String(data="Recogiendo objeto"))

            try:
                ''' Giro '''
                joints=self.robot.getActualQ()
                joints[5]+=request.angle
                self.robot.moveJ(joints)

                ''' Aproximación '''
                tcp_pose=self.robot.getActualTCPPose()
                tcp_pose[:3]=point
                pose_target=tcp_pose
                pose_tool=[0,0,-0.20,0,0,0]
                pose_approach = self.robot.poseTrans(pose_target,pose_tool)
                pose_approach[3:] = tcp_pose[3:]
                if pose_approach!=None:
                    if self.robot.moveJ_IK(pose_approach):
                        pose_catch = self.robot.poseTrans(pose_target, self.gripper_pose)
                        if pose_catch!=None:
                            if self.robot.moveL(pose_catch):
                                self.gripper.move_and_wait_for_pos(255, 10, 10) # Angle, Force, Speed

                                ''' Alejar '''
                                if self.robot.moveL(pose_approach):
                                    '''Reposo'''
                                    if self.robot.moveJ_IK(self.repose_pose, self.speed, self.acceleration):
                                        self.luces.change_color("VERDE")

                                        # ''' Giro a 0'''
                                        # joints=self.robot.getActualQ()
                                        # joints[5]=0
                                        # self.robot.moveJ(joints)
                                        ''' Recogida'''
                                        response.success = True
                                        print("[green]<pick_fruit>Fruta recogida!")
                                    else:
                                        response.success = False
                                else:
                                    response.success = False
                            else:
                                response.success = False
                        else:
                            response.success = False
                    else:
                        response.success = False
                else:
                    response.success = False

            except Exception as e:
                print(f"[red]<pick_fruit> ERROR -> {e}")
                print("[ref]<pick_fruit>Fruta no recogida")
                self.pub_text.publish(String(data="No he podido recoger el objeto"))

                '''Reposo'''
                self.robot.moveJ_IK(self.repose_pose, self.speed, self.acceleration)
                self.luces.change_color("VERDE")
                response.success = False

        return response
    
    """ ----------------------------------------------------------------
    SERVICIO: Dejar objeto en la posición de destino
    ---------------------------------------------------------------- """ 
    def place_fruit(self, request, response):
        print("[blue]--------------- PLACE -------------------")
        print(f"[yellow]<place_fruit>Dejando fruta en {self.target_pose}")
        self.pub_text.publish(String(data="Dejando objeto en destino"))

        try:

            ''' Aproximación '''
            tcp_pose=self.robot.getActualTCPPose()
            tcp_pose[:3]=self.target_pose[:3]
            pose_target=tcp_pose
            pose_tool=[0,0,-0.20,0,0,0]
            pose_approach=self.robot.poseTrans(pose_target,pose_tool)
            if pose_approach!=None:
                if self.robot.moveJ_IK(pose_approach):
            
                    ''' Colocar'''
                    if self.robot.moveL(self.target_pose):
                        self.gripper.move_and_wait_for_pos(0, 10, 10) # Abrir
                        print(f"[green]<place_fruit> Servicio completado con éxito")

                        response.success = True
                        response.message = ""
                    else:
                        response.success = False
                        response.message = "Movimiento no completado"
                else:
                    response.success = False
                    response.message = "Movimiento no completado"
            else:
                response.success = False
                response.message = "Movimiento no completado"

        except Exception as e:
            print(f"[red]<place_fruit> ERROR -> {e}")
            self.pub_text.publish(String(data="No he podido dejar el objeto en destino"))

            '''Reposo'''
            self.robot.moveJ_IK(self.repose_pose)
            response.success = False
            response.message = e
            
        return response
    
    """ ----------------------------------------------------------------
    SERVICIO: Reconectar
    ---------------------------------------------------------------- """ 
    def reconnect(self, request, response):
        if not self.robot.robot_connected:
            self.robot.robot_connected, _ = self.robot.rtde_connection()
            if self.robot.robot_connected:
                response.success = True
                response.message = "Conexión establecida"
            else:
                response.success = False
                response.message = "Conexión rechazada"
        else:
            response.success = False
            response.message = "El robot ya está conectado"
            
        return response
    
    """ ----------------------------------------------------------------
    SERVICIO: Transformar coordendas de la cámara a la referencia del robot
    ---------------------------------------------------------------- """ 
    def cam2robot(self,request,response):
        print("[blue]--------------- SERVICIO TRANSFORMACIÓN -------------------")
        print(f"[yellow]Recibido punto en referencia cámara: {request.position_in_ref1}")
        # Sacando la posición en este momento
        tcp_pose=self.robot.getActualTCPPose()
        M_tcp_rob=trs.pose2mth(tcp_pose)

        point_in_cam = np.array([request.position_in_ref1.x,request.position_in_ref1.y, request.position_in_ref1.z])
        point_in_tcp = trs.refA2refB(self.T_cam_rob, point_in_cam)
        print(f"[yellow]Punto en referencia TCP: {point_in_tcp}")
        point_in_robot = trs.refA2refB(M_tcp_rob, point_in_tcp)
        print(f"[yellow]Punto en referencia robot: {point_in_robot}")
        
        response.position_in_ref2.x, response.position_in_ref2.y, response.position_in_ref2.z = point_in_robot[0], point_in_robot[1], point_in_robot[2]
        return response


'''------------------------------------------------------------------------------------------------------------------------------------------------------'''
def main(args=None):
    rclpy.init(args=args)

    config = get_mongo_config()

    try:
        luces = claseLuces.TableLights(config["ip_lights"])
        robot = claseRobot.Robot(ip = config["ur10e"]["ip_address"], mode = config["ur10e"]["mode"], luces = luces, autoreconnection = True)
        if robot.robot_connected:
            gripper = clasePinza.RobotiqGripper(ip = config["ur10e"]["ip_address"])
            try:
                pick_node = PickRobot(config, robot, gripper, luces)

                # Mover a posición de reposo y abrir garra
                robot.moveJ_IK(pick_node.repose_pose, pick_node.speed, pick_node.acceleration)
                robot.set_repose_pose(pick_node.repose_pose)
                luces.change_color("VERDE")
                if robot.robot_connected:
                    gripper.activate()
                    gripper.move_and_wait_for_pos(0, 255, 255)
                rclpy.spin(pick_node)
            except Exception as e:
                pick_node.destroy_node()
                rclpy.shutdown()
    except Exception as e:
        print(f"[red][ERROR] -> {e}")
        traceback.print_exc()
    finally:
        luces.gateway_disconnection()
        if robot.robot_connected:
            robot.control_disconnection()
            robot.receive_disconnection()
            robot.stop_thread()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
