
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fruit_picking import get_mongo_config, claseRobot, claseLuces, trs, user
from my_robot_interfaces.msg import JXCP1Inputs
from my_robot_interfaces.srv import PickObject, FruitInfo, TransformCoordinates
from example_interfaces.srv import Trigger
import threading
from rich import print
import time
import traceback

class FruitManager(Node):
    def __init__(self, config):
        super().__init__('fruit_manager')
        self.config = config
        self.target_guide_position = self.config.get('plc').get('position_1')
        self.start_guide_position = self.config.get('plc').get('position_2')
        
        self.robot_in_process = False
        self.fruit = None
        self.guide_position = None
        self.place_available = False
        self.fruit_in_guide = False

        self.set_services()
        self.set_publishers_and_subscribers()
        
        print("[green]>>Nodo Fruit Manager inicializado")
        self.pub_text.publish(String(data = "Todo listo. ¿Qué fruta deseas recoger?"))

    def set_publishers_and_subscribers(self):
        self.create_subscription(String, 'fruit_picking/ollama/selected_fruit', self.ollamaCallback, 10)
        self.create_subscription(JXCP1Inputs, 'linear_guide/current_info', self.guideCallback, 10)
        self.pub_text = self.create_publisher(String, 'audio_interface/play_audio', 10)
    
    def set_services(self):
        clients = []
        self.client_info = self.create_client(FruitInfo, 'fruit_picking/camera/get_fruit_info')
        
        self.client_pick = self.create_client(PickObject, 'fruit_picking/robot/pick_object')
        self.client_place = self.create_client(Trigger, 'fruit_picking/robot/place_object')
        self.client_transform = self.create_client(TransformCoordinates, 'fruit_picking/robot/camera2robot')
        self.client_repose = self.create_client(Trigger, 'fruit_picking/robot/move2repose')
        self.client_guide_1 = self.create_client(Trigger, 'linear_guide/move_to_position_1')
        self.client_guide_2 = self.create_client(Trigger, 'linear_guide/move_to_position_2')

        self.check_services()
        

        self.info_request = FruitInfo.Request()
        self.robot_request = PickObject.Request()
        self.coordinates_request = TransformCoordinates.Request()

    def check_services(self):
        for cli in self._clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                print(f"[red]Servicio {cli.srv_name} no disponible")
    
    """ ----------------------------------------------------------------
    LLAMADAS A SERVICIOS
    ---------------------------------------------------------------- """ 
    # Obtener información de la fruta
    def send_request_info(self):
        self.info_request.fruit = self.fruit
        return self.client_info.call(self.info_request)
    
    # Llamada al pick 
    def send_request_pick(self, point, angle):
        self.robot_request.point = point
        self.robot_request.angle = angle
        return self.client_pick.call(self.robot_request)
    
    # Llamada al place
    def send_request_place(self):
        return self.client_place.call(Trigger.Request())
    
    # Llamada al servicio de transformadas
    def send_request_coordinates(self, point_in_cam):
        self.coordinates_request.position_in_ref1 = point_in_cam
        return self.client_transform.call(self.coordinates_request)

    # Llamada al reposo
    def send_request_repose(self):
        return self.client_repose.call(Trigger.Request())

    # Mover la guía
    def move_guide(self, position):
        if self.guide_speed==0:
            if position==1:
                return self.client_guide_1.call(Trigger.Request())
            elif position==2:
                return self.client_guide_2.call(Trigger.Request())
        else:
            print("[red]Peticicón rechazada. La guía se encuentra en movimiento")
            return False, "Peticicón rechazada. La guía se encuentra en movimiento"
        
    """ ----------------------------------------------------------------
    CALLBACKS
    ---------------------------------------------------------------- """ 
    # Recibir fruta mencionada
    def ollamaCallback(self, data):
        if not self.robot_in_process and self.fruit==None:
            self.fruit = data.data
        else:
            print("[red]<ollamaCallback>Petición rechazada.")
            print(f"[red]<ollamaCallback>El robot está en proceso de recogida de la fruta {self.fruit}")
    
    # Parámetros de la guía
    def guideCallback(self, info):
        self.guide_speed = info.current_speed
        self.guide_position = info.current_position
        # print(self.guide_position)
    
""" ----------------------------------------------------------------
MAIN
---------------------------------------------------------------- """ 
# Hilo de control de la guía
def check_guide(manager_node):
    while True:
        '''COMPROBAR POSICIÓN DE LA GUÍA'''
        if manager_node.guide_position == manager_node.start_guide_position: # Si está en la zona donde se hace el place
                # print("Place disponible")
                manager_node.place_available = True
                if manager_node.fruit_in_guide == True: # Si la fruta está colocada en la guía se mueve a la posición 1
                    manager_node.move_guide(1)

        # Cuando la guía no está en la posición de place
        else: 
            # print("Place no disponible")
            manager_node.place_available = False
            if manager_node.guide_position == manager_node.target_guide_position: # Si la guía está en la posición 1
                # print("[green]Fruta ha llegado a su destino")
                manager_node.fruit_in_guide = False
                time.sleep(1)
                # print("Moviendo guía posición de Place") # Se mueve la guía a posición de place
                manager_node.move_guide(2)

        # Comprobación cada 1 segundo     
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        config = get_mongo_config()
        robot_status = 0
        
        manager_node = FruitManager(config)

        # Hilo para callbacks y servicios
        # print("[blue]<main>Iniciando hilos...")
        spin_thread = threading.Thread(target=rclpy.spin, args=(manager_node,))
        spin_thread.start()
        # Hilo para control de la guía
        guide_thread = threading.Thread(target=check_guide, args=(manager_node,))
        guide_thread.start()

        try:   
            while rclpy.ok():
                # Comprobar si se ha mencionado una fruta
                if manager_node.fruit!=None and robot_status==0:
                    manager_node.robot_in_process = True

                    #TODO: Hacer descomposición de todas las frutas que llegan 
                    
                    # Llamada para obtener información de la fruta
                    print("[yellow]----------------------------------")
                    print(f"[yellow]Obteniendo información de {manager_node.fruit} ...")
                    fruit_info = manager_node.send_request_info()
                    print(f"Información obtenida:{fruit_info}")

                    # Llamada para mover el robot en las coordenadas indicadas 
                    if fruit_info.success:
                        if fruit_info.point.x != 0.0 and fruit_info.point.y != 0.0 and fruit_info.point.z != 0.0:
                            robot_coordinates = manager_node.send_request_coordinates(fruit_info.point)
                            print(f"[yellow]Ordenando a robot moverse a posición {robot_coordinates.position_in_ref2} y ángulo {fruit_info.angle}  ...")
                            # manager_node.pub_text.publish(String(data=f"Voy. Recogiendo el {manager_node.fruit}"))
                            robot_success = manager_node.send_request_pick(robot_coordinates.position_in_ref2, fruit_info.angle)
                            if robot_success.success:
                                print(f"[green]Servicio completado con éxito. Robot ha recogido {manager_node.fruit}")
                                robot_status = 1
                            else:
                                print(f"[red]Servicio completado sin éxito. Robot no ha recogido {manager_node.fruit}")
                                manager_node.robot_in_process = False
                                manager_node.fruit = None
                        else:
                            print(f"[red]Coordenadas de la cámara erróneas. No se puede recoger la fruta {manager_node.fruit}")
                            manager_node.robot_in_process = False
                            manager_node.fruit = None
                    else:
                        print(f"[red]<main>Error en la información recibida -> {fruit_info.error}")
                        manager_node.robot_in_process = False
                        manager_node.fruit = None
                    
                # Llamada al servicio para dejar la fruta en la guía cuando está en posición de Place si el robot está en proceso:
                # Esto no se hace de forma secuencial al pick, ya que se espera a que la guía esté disponible
                print()
                if manager_node.robot_in_process and manager_node.place_available and robot_status==1:
                    place_response = manager_node.send_request_place()
                    if place_response.success:
                        manager_node.fruit_in_guide = True
                        repose_response = manager_node.send_request_repose()
                        if repose_response.success:
                            print(f"[green]Servicio completado con éxito. Robot ha colocado la fruta {manager_node.fruit} en la cinta")
                            manager_node.robot_in_process =False
                            robot_status = 0
                            manager_node.fruit = None
                    
        finally:
            manager_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
