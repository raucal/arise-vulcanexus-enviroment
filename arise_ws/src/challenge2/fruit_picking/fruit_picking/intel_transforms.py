import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import Extrinsics
from fruit_picking import get_mongo_config, realsense2ros, trs
from my_robot_interfaces.srv import FruitInfo
from my_robot_interfaces.msg import Fruits
from geometry_msgs.msg import Vector3
from rich import print
import numpy as np
import pyrealsense2 as rs2
import cv2
import traceback

class IRSTransforms(Node):
    def __init__(self, config):
        super().__init__('irs_transforms')
        self.config = config
        self.T_cam_rob = config["ur10e"]["T_cam_tcp"]

        self.depth_min = self.config["camera_vision"]["depth_min"]
        self.depth_max = float(self.config["camera_vision"]["depth_max"])
        self.depth_scale = self.config["camera_vision"]["depth_scale"]

        self.bgr_image, self.depth_image, self.aligned_image  = None, None, None
        self.bgr_intrinsics, self.depth_intrinsics, self.aligned_intrinsics = rs2.intrinsics(), rs2.intrinsics(), rs2.intrinsics()
        self.depth2color_extrinsics, self.color2depth_extrinsics = rs2.extrinsics(), rs2.extrinsics()

        self.fruits_in_image = {}
        self.selected_fruit = None

        self.set_publishers_and_subscribers()
        self.set_servers()


    def set_publishers_and_subscribers(self):
        # Images
        self.create_subscription(Image, "/d435/camera/color/image_raw", self.bgrImageCallback, 10)
        self.create_subscription(Image, "/d435/camera/depth/image_rect_raw", self.depthImageCallback, 10)
        self.create_subscription(Image, "/d435/camera/aligned_depth_to_color/image_raw", self.alignedImageCallback, 10)

        # Intrinsics
        self.create_subscription(CameraInfo, "/d435/camera/color/camera_info", self.imageColorInfoCallback, 10)
        self.create_subscription(CameraInfo, "/d435/camera/depth/camera_info", self.imageDepthInfoCallback, 10)
        self.create_subscription(CameraInfo, "/d435/camera/aligned_depth_to_color/camera_info", self.imageAlignedInfoCallback, 10)

        # Depth Extrinsics
        self.create_subscription(Extrinsics,"/d435/camera/extrinsics/depth_to_color", self.depth2colorExtrinsicsCallback, 10)

        # Frutas
        self.create_subscription(Fruits, "/fruit_picking/camera/fruits_in_image", self.fruitsCallback, 1)

    def set_servers(self):
        self.create_service(FruitInfo, "/fruit_picking/camera/get_fruit_info", self.get_fruit_info)

    """ ----------------------------------------------------------------
    CALLBACKS: IMÁGENES
    ---------------------------------------------------------------- """ 
    def bgrImageCallback(self, image):
        bgr = realsense2ros.ros2array(image.data, image.height, image.width, np.uint8)
        self.bgr_image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def depthImageCallback(self, image):
        self.depth = realsense2ros.ros2array(image.data, image.height, image.width, np.uint16)
        depth_normalized = cv2.convertScaleAbs(self.depth, alpha=255.0/np.nanmax(self.depth))
        self.depth_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        # print("Depth actualizado: ", self.depth)

    def alignedImageCallback(self, image):
        aligned = realsense2ros.ros2array(image.data, image.height, image.width, np.uint16)
        aligned_normalized = cv2.convertScaleAbs(aligned, alpha=255.0/np.nanmax(aligned))
        self.aligned_image = cv2.applyColorMap(aligned_normalized, cv2.COLORMAP_JET)

    """ ----------------------------------------------------------------
    CALLBACKS: INTRINSICS
    ---------------------------------------------------------------- """ 
    def imageColorInfoCallback(self, info):
        new_intrinsics = self._set_intrinsics(info, rs2.distortion.inverse_brown_conrady)
        if not self._intrinsics_equal(new_intrinsics, self.bgr_intrinsics): # Comparar atributos de los objetos uno a uno
            self.bgr_intrinsics = new_intrinsics
            print("BGR Intrinsics updated:: ", self.bgr_intrinsics)

    def imageDepthInfoCallback(self, info):
        new_intrinsics = self._set_intrinsics(info, rs2.distortion.brown_conrady)
        if not self._intrinsics_equal(new_intrinsics, self.depth_intrinsics):
            self.depth_intrinsics = new_intrinsics
            print("Depth Intrinsics updated: ", self.depth_intrinsics)
    
    def imageAlignedInfoCallback(self, info):
        new_intrinsics = self._set_intrinsics(info, rs2.distortion.inverse_brown_conrady)
        if not self._intrinsics_equal(new_intrinsics, self.aligned_intrinsics):
            self.aligned_intrinsics = new_intrinsics
            print("Aligned Intrinsics updated: ", self.aligned_intrinsics)

    def _intrinsics_equal(self, intrinsics1, intrinsics2):
        if (intrinsics1.width != intrinsics2.width or
            intrinsics1.height != intrinsics2.height or
            intrinsics1.ppx != intrinsics2.ppx or
            intrinsics1.ppy != intrinsics2.ppy or
            intrinsics1.fx != intrinsics2.fx or
            intrinsics1.fy != intrinsics2.fy or
            intrinsics1.coeffs != intrinsics2.coeffs):
            return False
        return True

    def _set_intrinsics(self, info, model):
        intrinsics = rs2.intrinsics()
        intrinsics.width = info.width
        intrinsics.height = info.height
        intrinsics.ppx = info.k[2]
        intrinsics.ppy = info.k[5]
        intrinsics.fx = info.k[0]
        intrinsics.fy = info.k[4]
        intrinsics.model = model
        intrinsics.coeffs = [i for i in info.d]   

        return intrinsics

    """ ----------------------------------------------------------------
    CALLBACKS: DEPTH2COLOR EXTRINSICS
    ---------------------------------------------------------------- """ 
    # El nodo de la cámara solo publica una vez los parámetros extrínsecos. Hay 2 opciones: O modificar el nodo de la cámara para que los publique en bucle o
    # iniciar el nodo de la cámara después de todos los demás.
    def _extrinsics_equal(self,ext1, ext2):
        return (all([r1 == r2 for r1, r2 in zip(ext1.rotation, ext2.rotation)]) and
                all([t1 == t2 for t1, t2 in zip(ext1.translation, ext2.translation)]))

    def depth2colorExtrinsicsCallback(self, info):
        if self.depth2color_extrinsics != rs2.extrinsics():
            # Depth2Color
            self.depth2color_extrinsics.rotation=info.rotation
            self.depth2color_extrinsics.translation=info.translation

            print("Depth2Color Extrinsics: ", self.depth2color_extrinsics)

            MTH_depth_color=np.zeros((4,4))
            MTH_depth_color[:3,:3]=np.array(self.depth2color_extrinsics.rotation).reshape((3,3))
            MTH_depth_color[:3,3]=np.array(self.depth2color_extrinsics.translation).reshape((1,3))
            MTH_depth_color[3,:]=np.array([0,0,0,1])

            MTH_color_depth=np.linalg.inv(MTH_depth_color)
            
            # Color2Depth
            self.color2depth_extrinsics.rotation=tuple(MTH_color_depth[:3,:3].reshape(9,))
            self.color2depth_extrinsics.translation=tuple(MTH_color_depth[:3,3].reshape(3,))
            print("Color2Depth Extrinsics: ", self.color2depth_extrinsics)

    """ ----------------------------------------------------------------
    CALLBACK DE LAS FRUTAS EN LA IMAGEN
    ---------------------------------------------------------------- """ 
    def fruitsCallback(self, msg):
        self.fruits_in_image["green_apple"]=msg.green_apple
        self.fruits_in_image["red_apple"]=msg.red_apple
        self.fruits_in_image["banana"]=msg.banana
        self.fruits_in_image["lemon"]=msg.lemon
        self.fruits_in_image["orange"]=msg.orange
        self.fruits_in_image["peach"]=msg.peach
        self.fruits_in_image["pear"]=msg.pear
        self.fruits_in_image["pepper"]=msg.pepper
        self.fruits_in_image["kiwi"]=msg.kiwi
        self.fruits_in_image["angles"]=msg.angles
        # print("[red]Diccionario",self.fruits_in_image)
    
    """ ----------------------------------------------------------------
    FUNCIONES : Transformar píxeles a coordenadas 3D
    ---------------------------------------------------------------- """ 
    def pixels_to_3d(self, pixels):

        if (self.depth_image is not None and 
            not self._intrinsics_equal(self.bgr_intrinsics, rs2.intrinsics()) and 
            not self._intrinsics_equal(self.depth_intrinsics, rs2.intrinsics()) and 
            not self._extrinsics_equal(self.depth2color_extrinsics, rs2.extrinsics()) and 
            not self._extrinsics_equal(self.color2depth_extrinsics, rs2.extrinsics())):

            try:
                depth_pixel = realsense2ros.rs2_project_color_pixel_to_depth_pixel(
                    self.depth, self.depth_scale, self.depth_min, self.depth_max, 
                    self.depth_intrinsics, self.bgr_intrinsics, 
                    self.color2depth_extrinsics, self.depth2color_extrinsics, pixels)
                
                depth = self.depth[int(depth_pixel[1]), int(depth_pixel[0])] / 1000.0  # meters
                P = realsense2ros.rs2_deproject_pixel_to_point(self.depth_intrinsics, depth_pixel, depth)
                
                cam_coordinates = [float(P[0]), float(P[1]), float(P[2])]
                output = None
            except Exception as e:
                output = "Error en el cálculo de coordenadas 3D de la cámara"
                print(f"[red]<pixels_to_3d> ERROR -> {e}")
                print(f"[orange1]{output} ")
                # print(traceback.format_exc())
                cam_coordinates = [0.0, 0.0, 0.0]
        else:
            output = "Algunos de los parámetros no están definidos. Vuelve a iniciar el nodo de la cámara"
            cam_coordinates = [0.0, 0.0, 0.0]

        return cam_coordinates, output

    """ ----------------------------------------------------------------
    SERVICIO: Devolver coordenadas 3D y ángulo de la fruta
    ---------------------------------------------------------------- """ 
    def get_fruit_info(self, request, response):
        print("[yellow]----------------------------------")
        self.selected_fruit = request.fruit # string
        print(f"[yellow]<get_fruit_info>Calculando datos de {self.selected_fruit}")

        try:
            ''' Comprobar si la fruta está en la imagen '''
            if self.fruits_in_image!={}:
                if self.fruits_in_image[self.selected_fruit][0]!=0 and self.fruits_in_image[self.selected_fruit][1]!=0:

                    ''' Obtener datos del diccionario'''
                    index = list(self.fruits_in_image.keys()).index(self.selected_fruit) # índice para saber la posición en los ángulos
                    pixels = self.fruits_in_image[self.selected_fruit]
                    angle = self.fruits_in_image["angles"][index]
                    print(f"[blue]<get_fruit_info>Pixeles: {pixels} Ángulo: {angle}")

                    ''' Convertir píxeles a coordenadas 3D '''
                    if not np.array_equal(pixels, (0, 0)):
                        cam_coordinates, output = self.pixels_to_3d(pixels) # Convertir píxeles a 3D en la referencia de la cámara
                        mean_z = self._calculate_depth(pixels)
                        print(f'[blue]<get_fruit_info> media de Z: {mean_z}')
                        print(f'[blue]<get_fruit_info> Punto en coordenadas 3D: {cam_coordinates}')
                        # Sustituir cam_coordinates[2] por mean_z
                        cam_coordinates[2] = mean_z
                        print(f'[blue]<get_fruit_info> Punto en coordenadas 3D (modificado): {cam_coordinates}')
                        if output==None:
                            response.success = True
                            response.error = "Información cargada"
                            response.point = Vector3()
                            response.point.x, response.point.y, response.point.z  = cam_coordinates[0], cam_coordinates[1], cam_coordinates[2]
                            response.angle = float(angle)
                        else:
                            response = self._wrong_response(output, response)
                    else:
                        print("[red]<get_fruit_info> Coordenadas no válidas")
                        response = self._wrong_response("La fruta elegida no ha sido detectada", response)
                else:
                    print("[red]<get_fruit_info> La fruta elegida no ha sido detectada")
                    response = self._wrong_response("La fruta elegida no ha sido detectada", response)
            else:
                print("[red]<get_fruit_info> No hay información sobre las frutas. Cargue el nodo de de detección")
                response = self._wrong_response("No hay información sobre las frutas. Cargue el nodo de detección", response)
        except Exception as e:
            print(f"[red]<get_fruit_info> [ERROR] -> {e}")
            # print(traceback.format_exc())
            response = self._wrong_response("No se reconoce ninguna fruta en la petición", response)

        return response

    def _wrong_response(self, error_message, response):
        response.success = False
        response.error = error_message
        response.point = Vector3()
        response.angle = 0.0
        return response   
    
    def _calculate_depth(self, pixels):
        """
        Esta función toma una coordenada de píxel (x, y), convierte los píxeles vecinos a coordenadas 3D,
        y devuelve la media de los valores z de esas coordenadas, excluyendo los puntos que sean [0.0, 0.0, 0.0].
        
        :param pixels: Tuple de coordenadas del píxel (x, y)
        :return: Media de los valores z de las coordenadas 3D de los píxeles vecinos
        """
        x, y = pixels
        
        # Vecinos cercanos: arriba, abajo, izquierda, derecha
        vecinos = [
            (x + 1, y),  # Derecha
            (x - 1, y),  # Izquierda
            (x, y + 1),  # Abajo
            (x, y - 1)   # Arriba
        ]
        
        
        # Lista para almacenar los valores z de las coordenadas 3D de los vecinos
        depth_values = []
        
        for vecino in vecinos:
            cam_coordinates, _ = self.pixels_to_3d(vecino)
            if not np.array_equal(cam_coordinates, [0.0, 0.0, 0.0]):
                depth_values.append(cam_coordinates[2])

        print('depth_values: ',depth_values)

        # Calcular la media de los valores z
        if depth_values:
            mean_depth = np.mean(depth_values)
        else:
            mean_depth = None  # o algún valor por defecto si no hay vecinos válidos
        
        return mean_depth


    def _load_last_extrinsics(self):
        self.depth2color_extrinsics.rotation = self.config["camera_vision"]["extrinsics"]["depth2color"]["rotation"]
        self.depth2color_extrinsics.translation = self.config["camera_vision"]["extrinsics"]["depth2color"]["translation"]
        self.color2depth_extrinsics.rotation = self.config["camera_vision"]["extrinsics"]["color2depth"]["rotation"]
        self.color2depth_extrinsics.translation = self.config["camera_vision"]["extrinsics"]["color2depth"]["translation"]
        print("Depth2Color Extrinsics: ", self.depth2color_extrinsics)
        print("Color2Depth Extrinsics: ", self.color2depth_extrinsics)

'''------------------------------------------------------------------------------------------------------------------------------------------------------'''

def main(args=None):
    rclpy.init(args=args)

    config = get_mongo_config()

    try:
        camera_node = IRSTransforms(config)
        camera_node._load_last_extrinsics()
        try:
            rclpy.spin(camera_node)
        except KeyboardInterrupt:
            camera_node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"[red]ERROR -> {e}")

if __name__ == '__main__':
    main()
