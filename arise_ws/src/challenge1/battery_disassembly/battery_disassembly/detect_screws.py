import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from my_robot_interfaces.srv import List, CheckInt
from battery_disassembly import get_mongo_config, user
# from cv_bridge import CvBridge

from ultralytics import YOLO  # Importamos funcionalidades de YOLO
from ultralytics.yolo.utils.torch_utils import select_device
import torch
import cv2
import numpy as np
from rich import print
import time

class DetectScrews(Node):
    def __init__(self, config):
        super().__init__('detect_screws') 
        self.config = config
        self.last_image = Image()
        self.threshold = self.config["camera_vision"]["threshold"]
        self.camera_name = self.config["camera_vision"]["camera_name"]
        # HACK: Descomentar cuando la cámara funcione 
        self.indexes = []
        # self.indexes =[1,2,3,4,5,6,7]

        # Imagen procesada
        self.m_img = Image()
        self.m_img.height = self.config["camera_vision"]["resolution"]["height"]
        self.m_img.width = self.config["camera_vision"]["resolution"]["width"]
        self.m_img.step = 3 * self.m_img.height
        self.m_img.encoding = 'rgb8'

        # self.br = CvBridge()

        # Arrancar el modelo
        model_path = '/home/'+user+'/'+'arise-vulcanexus-enviroment/'+self.config["camera_vision"]["model_path"]
        self.run_model(model_path)

        # Iniciar Comunicadores
        self.set_publishers_and_subscribers()
        # self.set_servers()  # YA NO SE UTILIZAN ESTOS SERVICIOS EN EL FLUJO DEL PROGRAMA

        # Hilo para procesar la imagen
        self.create_timer(0.01, self.timer_callback)
        self.create_timer(3, self.publish_image)
    
    def set_publishers_and_subscribers(self):
        if self.camera_name == "intel":
            self.create_subscription(Image, 'camera/color/image_raw', self.BGRCallback, 1) 
        elif self.camera_name == "luxonis":
            self.create_subscription(Image, '/oak/rgb/image_raw', self.BGRCallback, 1)
        else:
            print("[orange1]No se ha asignado ninguna cámara")
        self.pub_indexes = self.create_publisher(Int32MultiArray, 'battery_disassembly/camera/screws_in_image', 1)  # Indexes
        self.pub_image = self.create_publisher(Image, '/battery_disassembly/camera/processed_image', 1)
    
    """ ----------------------------------------------------------------
    ARRANCAR MODELO
    ---------------------------------------------------------------- """ 
    def run_model(self, model_path):
        # print(model_path)
        self.model = YOLO(model_path)  # YOLOV8
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        device = select_device(device)

        self.model.to(device)
        print(f"-> DEVICE: {device}")
    
    """ ----------------------------------------------------------------
    TRATAMIENTO IMAGEN
    ---------------------------------------------------------------- """ 
    # Recepción de la imagen de la cámara
    def BGRCallback(self, image):
        self.last_image = image
    
    # Detección de los tornillos
    def _cajas_yolo(self, bgr_frame, results):
        lista_indices= []
        ##############  ROI ##########################
        for result in results.boxes.data.tolist():
            X1, Y1, X2, Y2, score, class_id = result
            # print('score: ',score)
            x1, x2, y1, y2 =  int(X1), int(X2), int(Y1), int(Y2)
            coord = [int(X1), int(Y1)]

            screw_data = {
                ((1062, 51, x1, y1, x2, y2)): 1,  
                ((693, 55, x1, y1, x2, y2)): 2,  
                ((566, 55, x1, y1, x2, y2)): 3,
                ((190, 55, x1, y1, x2, y2)): 4,
                ((161, 659, x1, y1, x2, y2)): 5,
                ((598, 648, x1, y1, x2, y2)): 6,
                ((675, 647, x1, y1, x2, y2)): 7,
                ((1100, 638, x1, y1, x2, y2)): 8  
            }
                
            font = cv2.FONT_HERSHEY_SIMPLEX
            w = x2 - x1
            font_scale = 0.6
            thickness = 2
            texto = "1"
            text_size, _ = cv2.getTextSize(texto, font, font_scale, thickness)
            text_x = x1 + w // 2 - text_size[0] // 2
            text_y = y1 - text_size[1]
            grosor = 3
            
            for coordinates, screw_number in screw_data.items():
                lim_x, lim_y, x1, y1, x2, y2 = coordinates
                cv2.circle(bgr_frame, (lim_x, lim_y), grosor, (0, 0, 255), -1)

                if score > self.threshold:
                    if x1 < lim_x < x2 and y1 < lim_y < y2:
                        lista_indices.append(screw_number)
                        cv2.rectangle(bgr_frame, (int(X1), int(Y1)), (int(X2), int(Y2)), (0, 255, 0), 4)
                        cv2.putText(bgr_frame, str(screw_number), (text_x, text_y), font, font_scale, (0, 0, 255), thickness)

        return lista_indices, bgr_frame

    # Hilo Procesar
    def timer_callback(self):
        if self.last_image != Image():
            # Guardar imagen BGR
            img_bgr = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(self.last_image.height, self.last_image.width, -1)
            img_bgr = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            # Procesar la imagen
            results = self.model(img_bgr)[0]
            self.indexes, img_bgr = self._cajas_yolo(img_bgr, results)
            
            self.indexes.sort()  # Ordenar los índices
            # print("Indices: ",self.indexes)

            # Publicar lista de índices de los tornillos detectados
            self.screws_msg = Int32MultiArray(data=self.indexes)
            self.pub_indexes.publish(self.screws_msg)

            # Convertir imagen procesada al formato de ROS 2
            if self.camera_name == "intel":
                img_bgr = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2BGR)
            self.m_img.data = img_bgr.reshape(-1).tolist() 
            self.m_img.header.stamp = self.get_clock().now().to_msg()  # Indica cuando se genera el mensaje
            # self.pub_image.publish(self.m_img)  
        else:
            print("[red] ⚠️   ADVERTENCIA ⚠️: No se detecta ninguna cámara")
            time.sleep(5)
            
    
    def publish_image(self):
        if self.m_img != Image():
            self.pub_image.publish(self.m_img)  

def main(args=None):
    rclpy.init(args=args)

    config = get_mongo_config()

    camera_node = DetectScrews(config)

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        print("Interrupción recibida, cerrando el nodo...")
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()
        print("Nodo cerrado correctamente.")

if __name__ == '__main__':
    main()
