'''Programa para realizar la detección y localización de los tornillos.'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from fruit_picking import get_mongo_config, user
from my_robot_interfaces.msg import Fruits

from ultralytics import YOLO #Importamos funcionalidades de YOLO
from ultralytics.yolo.utils.torch_utils import select_device
import torch
import cv2
import numpy as np
from rich import print
import math

class DetectFruits(Node):
    def __init__(self, config):
        super().__init__('detect_fruits') 
        self.config = config
        self.last_image = Image()
        self.threshold = self.config["camera_vision"]["threshold"]

        # Imagen procesada
        self.m_img = Image()
        self.m_img.height = self.config["camera_vision"]["resolution"]["height"]
        self.m_img.width = self.config["camera_vision"]["resolution"]["width"]
        self.m_img.step = 3 * self.m_img.height
        self.m_img.encoding = 'rgb8'

        # Frutas
        self.fruits = Fruits()

        # Arrancar el modelo
        model_path = '/home/'+user+'/'+'arise-vulcanexus-enviroment/'+self.config["camera_vision"]["model_path"]
        self.run_model(model_path)

        # Iniciar Comunicadores
        self.set_publishers_and_subscribers()
        # self.set_servers() # YA NO SE UTILIZAN ESTOS SERVICIOS EN EL FLUJO DEL PROGRAMA

        # Hilo para procesar la imagen
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_timer(1, self.publish_image)
    
    def set_publishers_and_subscribers(self):
        self.create_subscription(Image, 'd435/camera/color/image_raw', self.BGRCallback, 1)
        self.pub_fruits = self.create_publisher(Fruits, 'fruit_picking/camera/fruits_in_image', 1) 
        self.pub_image = self.create_publisher(Image, 'fruit_picking/camera/processed_image',10)
    
    # En vez de crear un servidor se puede crear un topic que este publicando todo el rato los tornillos que se ven
    def set_servers(self):
        pass

    """ ----------------------------------------------------------------
    ARRANCAR MODELO
    ---------------------------------------------------------------- """ 
    def run_model(self, model_path):
        self.model = YOLO(model_path) #YOLOV8
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        device = select_device(device)
        self.model.to(device)
        print(f"-> DEVICE: {device}")

    """ ----------------------------------------------------------------
    FILTROS
    ---------------------------------------------------------------- """ 
    def f_FILTROS(self, box_ord,img):
        img_h, img_w, _ = img.shape
        img_area = int(img_h * img_w)
        new_box_ord = list()
        for box in box_ord:
            w_box = box[2]
            h_box = box[3]
            box_area = int(w_box * h_box)
            # print('Area de la caja: ', box_area)
            # print('-----------------------')
            # print('dif relativa: ', box_area / img_area)
            if box_area / img_area < 0.8:
                new_box_ord.append(box)
        return new_box_ord

    def f_FILTRO_margen(self, box_ord, img):
        new_box_ord = list()
        img_h, img_w, _ = img.shape
        i_1 = [0, 0]
        i_2 = [img_w, 0]
        i_3 = [0, img_h]
        i_4 = [img_w, img_h]
        for box in box_ord:
            x1 = box[0]
            y1 = box[1]
            w_box = box[2]
            h_box = box[3]
            b_1 = [
            x1, y1]
            b_2 = [int(x1 + w_box), y1]
            b_3 = [x1, int(y1 + h_box)]
            b_4 = [int(x1 + w_box), int(y1 + h_box)]
            if i_1 == b_1 or i_2 == b_2 or i_3 == b_3 or i_4 == b_4:
                ...
            else:
                new_box_ord.append(box)

        return new_box_ord

    def f_FILTRO_CONF(self, box_ord, conf_threshold):
        new_box_ord = list()
        for box in box_ord:
            conf = box[6]
            print(f"🛈  Confianza de la caja: {conf}")
            if conf_threshold < conf:
                new_box_ord.append(box)
        return new_box_ord

    """ ----------------------------------------------------------------
    CALCULAR PROOPIEDADES
    ---------------------------------------------------------------- """ 
    def calcular_angulo(self, start_point, end_point):
        # Calcular las diferencias en las coordenadas x e y
        delta_x = end_point[0] - start_point[0]
        delta_y = end_point[1] - start_point[1]

        # Calcular el ángulo en radianes usando atan2
        angle_rad = math.atan2(delta_y, delta_x)

        # Convertir el ángulo a grados
        angle_deg = math.degrees(angle_rad)
        # print(f'[dark_blue]🛈 FUNCION Angulo ºC: {angle_deg} Angulo rad: {angle_rad}')

        return angle_rad
    
    def calcular_centroide(self, mask):
        # Calcular momentos de la máscara
        moments = cv2.moments(mask)
        if moments['m00'] != 0:
            centroid_x = int(moments['m10'] / moments['m00'])
            centroid_y = int(moments['m01'] / moments['m00'])
            centroid = (centroid_x, centroid_y)
            # print("[blue]🛈 Centroide de la máscara: ", centroid)
            return centroid
        else:
            print("[yellow]⚠️ La máscara está vacía.")
            return None 

    def calcular_pts_medios(self, oriented_bb,oriented_img):
        puntos_medios=[]
        for i in range(0,len(oriented_bb)):
            center = oriented_bb[i][1]
            angle = oriented_bb[i][2]
            
            p1=oriented_bb[i][0][0]                                  
            p2=oriented_bb[i][0][1]  
            p3=oriented_bb[i][0][2]  
            p4=oriented_bb[i][0][3]  

            A = p1
            B = p2
            C = p4
            D = p3

            # PUNTOS MEDIOS borde
            """LEFT-TOP"""
            Z = [int((B[0]-A[0])/2), int((B[1]-A[1])/2)]
            Z = [Z[0]+A[0],Z[1]+A[1]]

            """LEFT-BOT"""
            H = [int((C[0]-A[0])/2), int((C[1]-A[1])/2)]
            H = [H[0]+A[0],H[1]+A[1]]


            """RIGHT-TOP"""
            I = [int((D[0]-B[0])/2), int((D[1]-B[1])/2)]
            I = [I[0]+B[0],I[1]+B[1]]


            """RIGHT-BOT"""
            J = [int((C[0]-D[0])/2), int((C[1]-D[1])/2)]
            J = [J[0]+D[0],J[1]+D[1]]


            # VISUALIZAR
            cv2.circle(oriented_img,(Z),1,(0,0,200),3)
            cv2.putText(oriented_img, str('Z'), (int(Z[0]),int(Z[1]-25)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255, 255, 255), 1, cv2.LINE_AA)

            cv2.circle(oriented_img,(H),1,(0,0,200),3)
            cv2.putText(oriented_img, str('H'), (int(H[0]),int(H[1]-25)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255, 255, 255), 1, cv2.LINE_AA)

            cv2.circle(oriented_img,(I),1,(0,0,200),3)
            cv2.putText(oriented_img, str('I'), (int(I[0]),int(I[1]-25)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255, 255, 255), 1, cv2.LINE_AA)

            cv2.circle(oriented_img,(J),1,(0,0,200),3)
            cv2.putText(oriented_img, str('J'), (int(J[0]),int(J[1]-25)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255, 255, 255), 1, cv2.LINE_AA)

            # PUNTOS MEDIOS OBJETO
            #dividimos los vecotres en n tramos
            n = 20
            HI = [int((I[0]-H[0])/n), int((I[1]-H[1])/n)]
            Hobj = [H[0]+HI[0],H[1]+HI[1]]
            Iobj = [I[0]-HI[0],I[1]-HI[1]]

            ZJ = [int((J[0]-Z[0])/n), int((J[1]-Z[1])/n)]
            Zobj = [Z[0]+ZJ[0],Z[1]+ZJ[1]]
            Jobj = [J[0]-ZJ[0],J[1]-ZJ[1]]

            # VISUALIZAR
            cv2.circle(oriented_img,(Hobj),1,(255,255,255),3)
            cv2.circle(oriented_img,(Iobj),1,(255,255,255),3)
            cv2.circle(oriented_img,(Zobj),1,(255,255,255),3)
            cv2.circle(oriented_img,(Jobj),1,(255,255,255),3)

            puntos_medios.append([Zobj,Hobj,Iobj,Jobj])

        # cv2.imshow('puntos',oriented_img)
        # cv2.waitKey(0)

        return puntos_medios

    def puntos_anclaje(self, oriented_bb,bgr,puntos_medios,masks_image_list):
        puntos_anclaje_list=[]
        for i in range(0,len(oriented_bb)):
            Zobj,Hobj,Iobj,Jobj = puntos_medios[i]

            center = oriented_bb[i][1]

            # Calcular la distancia euclidiana
            distancia_HI = np.sqrt(np.sum((np.array(Hobj) - np.array(Iobj))**2))
            distancia_ZJ = np.sqrt(np.sum((np.array(Zobj) - np.array(Jobj))**2))

            if distancia_HI<distancia_ZJ:
                # dibujamos una linea en la imagen
                start_point = Hobj
                end_point = Iobj
                        
            else:
                # Dibujamos una linea en la imagen
                start_point = Zobj
                end_point = Jobj

            puntos_anclaje = np.array([start_point, end_point, center])
            # print('\npuntos anclaje',puntos_anclaje)
            cv2.line(bgr, start_point, end_point, (200,200,200), 1)  #DIBUJAMOS UNA LINEA  
            
            mask = masks_image_list[i][0]        
            contornos, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

            # Crear una imagen en blanco del mismo tamaño que la máscara
            mask_line = np.zeros_like(mask)
            mask_cont = np.zeros_like(mask)
            # Dibujar la línea en la imagen en blanco
            cv2.line(mask_line, tuple(map(int, start_point)), tuple(map(int, end_point)), 255, 1)

            if contornos:
                # Realizar la operación AND entre la máscara y la línea
                # Dibujar los contornos en la imagen en blanco
                cv2.drawContours(mask_cont, contornos, -1, 255, thickness=1)
                cv2.drawContours(bgr, contornos, -1, (255,0,0), thickness=1)

                result = cv2.bitwise_and(mask_cont, mask_line)

                # Encontrar los puntos que no son cero (puntos de intersección)
                points_of_intersection = cv2.findNonZero(result)            
                # print('\npoints_of_intersection',points_of_intersection)

                # Si hay puntos de intersección, actualizar start_point y end_point con el punto más cercano
                if points_of_intersection is not None and len(points_of_intersection) > 0:
                    "Comporbar si hay algun punto de interseccion igual al stat/end point"
                    points_to_remove = []                
                    for i, point in enumerate(points_of_intersection):
                        point = point[0]  # Accede a las coordenadas directamente
                        if np.array_equal(point, start_point) or np.array_equal(point, end_point):
                            print('[yellow]\n⚠️  Interseccion igual a end/start point. Marcando punto para eliminación.')
                            points_to_remove.append(i)

                    if points_to_remove:
                        points_of_intersection = np.delete(points_of_intersection, points_to_remove, axis=0)

                    # print('\n[bold green]Points of intersection AFTAR REMOVE',points_of_intersection)

                    if len(points_of_intersection) > 0:
                        for i in range(0,len(points_of_intersection)):
                            p_of_intersection = points_of_intersection[i]
                            # print(f'\n[bold dark_orange]Point {i}: {p_of_intersection}') 

                            # points_of_intersection=points_of_intersection[0]    

                            # print('\n[bold bright_magenta]points_of_intersection red',points_of_intersection)         
                        
                            distances_start = np.sqrt(np.sum((np.array(p_of_intersection) - np.array(start_point))**2))
                            distances_end = np.sqrt(np.sum((np.array(p_of_intersection) - np.array(end_point))**2))
                            # print(f'[bold yellow2]distances_start {distances_start} distances_end {distances_end}')
                            if distances_start < distances_end:
                                
                                start_point = p_of_intersection[0]
                                # print(f'salidas {start_point} | {end_point} | {center}')
                                puntos_anclaje = np.array([p_of_intersection[0], end_point, center])
                                
                                # print(f'[bold sea_green1]BUCLE {i}')
                            else:
                                end_point = p_of_intersection[0]
                                # print(f'entradas {start_point} | {p_of_intersection[0]} | {center}')
                                puntos_anclaje = np.array([start_point, p_of_intersection[0], center])
                                # print(f'[bold sea_green2]BUCLE {i}')

            # print('\n[yellow]p anclaje fin',puntos_anclaje)
            # Dibujar los puntos de anclaje en la imagen
            for punto in puntos_anclaje:
                cv2.circle(bgr, (punto[0],punto[1]), 3, (0, 165, 255), -1)  # Radio del círculo: 3
            cv2.circle(bgr,(center),1,(255,255,255),3)
            puntos_anclaje_list.append(puntos_anclaje)

        # cv2.imshow('Puntos de anclaje',bgr)
        # cv2.waitKey(0)

        return puntos_anclaje_list,bgr
 
    # Dibuja las bounding box y los centros en la imagen
    def dibujar_cajas(self, boxes_list,img_with_marks):

        # Diccionario de colores asociados a cada fruta
        colores = {
            'Red Apple': (255, 0, 0),   # Rojo
            'Green Apple': (0, 255, 0),    # Verde
            'Banana': (0, 200, 200),  # Amarillo más cálido
            'Lemon': (128, 255, 0),  # Amarillo verdoso
            'Orange': (0, 165, 255),  # Naranja
            'Peach': (255, 182, 255),  # Rosa pálido
            'Pear': (0, 128, 0),    # Verde oscuro
            'Pepper': (128, 0, 0),    # Granate
            'Kiwi': (139, 69, 19)   # Verde/marrón
        }

        for i in range(0,len(boxes_list)):# Repasamos todas las cajas
            x,y,w,h,center=boxes_list[i][0],boxes_list[i][1],boxes_list[i][2],boxes_list[i][3],boxes_list[i][4]
            nombre_fruta = boxes_list[i][7]
            
            # print('nombre',nombre_fruta)

            color_asociado = colores.get(nombre_fruta, (0, 0, 0))  # Si la clase no está en el diccionario, retorna negro por defecto
            # print(f'[red] x y h w: {color_asociado}')
            img_with_marks = cv2.rectangle(img_with_marks, (x,y), (x+w,y+h), color_asociado, 2) #Bounding box de la caja
            img_with_marks = cv2.circle(img_with_marks, center, 2, color_asociado, -1) # Centros

            # Dibuja el nombre de la fruta
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            text_size = cv2.getTextSize(nombre_fruta, font, font_scale, font_thickness)[0]
            text_position = (x, y - 5)

            # Dibuja el recuadro azul
            box_color = color_asociado
            rect_start = (x, y)
            rect_end = (x + text_size[0], y - 5 - text_size[1])
            img_with_marks = cv2.rectangle(img_with_marks, rect_start, rect_end, box_color, -1)


            img_with_marks = cv2.putText(
                img_with_marks,
                nombre_fruta,
                text_position,
                font,
                font_scale,
                (255, 255, 255),
                font_thickness,
                cv2.LINE_AA,
            )
        
    def segment_roi(self, boxes_list,results,bgr_frame):
        masks_list = []
        for i in range(0,len(boxes_list)):# Repasamos todas las cajas
            # Creamos el ROI
            x,y,w,h,center=boxes_list[i][0],boxes_list[i][1],boxes_list[i][2],boxes_list[i][3],boxes_list[i][4]
            roi = bgr_frame[y:y+h,x:x+w]
            # cv2.imshow('roi',roi)
            # cv2.waitKey(1)
            # print('roi shape',roi.shape)
            
            # Cargamos el modelos de segmentacion
            for r in results:
                
                masks = r.masks  # Masks object for segmentation masks outputs
                # print('\n[bold dodger_blue3]INFO: ',r.boxes[i].cls.item())            

                if masks is None:
                    print('[red]❌ Error segmentacion. mascara is none\n')
                    mascara = np.zeros_like(roi, dtype=np.uint8)
                    mascara_reshaped = np.reshape(mascara, (mascara.shape[0] * mascara.shape[1], mascara.shape[2]))
                    # Tomar la primera dimensión
                    mascara_reshaped = mascara_reshaped[:, 0]
                    # print('\n mascara vacia: ',mascara_reshaped)
                    masks_list.append(mascara)
                    # cv2.imshow('Mask Image none', mascara)
                    # cv2.waitKey(1)
                    
                else:
                    # print('LONGITUD MASCARA ',len(masks))
                    "mask[0] se pone por si hay algun caso que el mode de segementacion vea dos mascaras en una sola imagen"
                    # for mask in masks:
                    # Convierte el tensor a una matriz NumPy                
                    m = masks[i][0].data[0].cpu().numpy()  # Utiliza .cpu() para copiar el tensor a la memoria del host
                    # print('\nmasks: ',m)
                    mask_cont = masks[0].xy[0]       
                    # print('\n lo que veo en mask.data: ',masks[0].data[0].numpy())                
                    #FIXME
                    w,h = masks[0].orig_shape
                    # print(f'w: {w}, h {h} de la mascara')
                    
                    mask_np = np.array(m, dtype=np.uint8)
                    # Reshape al tamaño original
                    mask_np = cv2.resize(mask_np,(h,w))

                    # mask_np = mask_np.reshape(w,h)

                    # Encuentra los contornos en la máscara limpiada
                    contornos, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # Crea una máscara en blanco del mismo tamaño que la original
                    mascara = np.zeros_like(mask_np, dtype=np.uint8)

                    # Dibuja todos los contornos internos en la nueva máscara
                    cv2.drawContours(mascara, contornos, -1, 255, thickness=cv2.FILLED)

                    # print(f'\nmascara_n`p 640x640: {mask_np}\nmascara reshape{mascara}')

                    mask_data =[mascara]
                    # print(f'\n[cyan] añado la mascara {mask_data} en el bucle {i}')
                    masks_list.append(mask_data)

                    # # Muestra la máscara con cv2.imshow
                    # cv2.imshow(f'Mask Image {i}', mascara)
                    # cv2.waitKey(0)

        # print('[yellow] len lista de mascaras:',len(masks_list))
        # print('[yellow] lista de mascaras:',masks_list)
        return masks_list            

    def get_oriented_bb(self, boxes_list,bgr,masks_list):

        imagen_th = bgr.copy()
        oriented_bounding_boxes = []

        """1.  SACAMOS EL ROI DE CADA CAJA"""
        for i in range(0,len(boxes_list)):
            x,y,w,h = boxes_list[i][0],boxes_list[i][1],boxes_list[i][2],boxes_list[i][3]

            """2.  DETECTAMOS EL CONTORNO"""
            mask = masks_list[i][0]    

            # Encuentra los contornos en la máscara limpiada
            contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   #cv2.RETR_CCOMP
                    
            centroide = self.calcular_centroide(mask)        

            if centroide is not None:
                # centroide = centroide + np.array([x, y])
                # print(centroide)

                # Asegúrate de que haya al menos un contorno detectado
                if contornos:
                    # Encuentra el contorno con el área máxima
                    contorno_maximo = max(contornos, key=cv2.contourArea)

                    # Angulo
                    _, _, angle = cv2.fitEllipse(contorno_maximo)
                    angle_rad = np.deg2rad(angle)
                    # print(f'[blue]🛈 Angulo ºC: {angle} Angulo red: {angle_rad}')
                    
                    # Obtén el rectángulo de área mínima para el contorno máximo
                    rect = cv2.minAreaRect(contorno_maximo)
                    
                    # Dibuja el rectángulo en la imagen original
                    OBbox = cv2.boxPoints(rect)
                    OBbox = np.intp(OBbox)
                    
                    # Ajusta las coordenadas para que correspondan a la imagen original completa
                    # OBbox = OBbox + np.array([x, y])

                    cv2.drawContours(imagen_th, [OBbox], 0, (250, 0, 0), 2)
                    cv2.circle(imagen_th, centroide, 1, (255, 0, 0), -1)  # Dibuja un círculo verde en el centroide
                    # cv2.imshow('Orientada', imagen_th)

                    boxes_data=[OBbox,centroide,angle] 
                    oriented_bounding_boxes.append(boxes_data)
                    print('\n')
                else:
                    print("[yellow]⚠️  No se encontraron contornos en la máscara.")
                    cv2.destroyAllWindows()
            else:
                print("[yellow]⚠️  El centroide es [purple]None. [yellow]No se puede realizar la operación.\n")
        
        return oriented_bounding_boxes, imagen_th

    """ ----------------------------------------------------------------
    TRATAMIENTO IMAGEN
    ---------------------------------------------------------------- """ 
    # Recepción de la imagen de la cámara
    def BGRCallback(self,image):
        self.last_image = image
    
    # Detección de los tornillos
    def _cajas_yolo(self,bgr_frame,results):
        boxes_list = [] #Definimos lista donde guardaremos las cajas de detección
        frame_h, frame_w, frame_ch = bgr_frame.shape
        frame_center = [int(frame_w/2),int(frame_h/2)]
        names = {0: 'Red Apple', 
        1: 'Green Apple', 
        2: 'Banana', 
        3: 'Lemon',  
        4: 'Orange',
        5: 'Peach', 
        6: 'Pear', 
        7: 'Pepper',       
        8: 'Kiwi'}
        
        for r in results:  
            boxes = r.boxes
            # print('\nr: ',r)
            for box in boxes:
                b = box.xyxy[0].tolist()  # get box coordinates in (top, left, bottom, right) format
                clase = box.cls.item()
                nombre_fruta = names.get(clase,'Desconocido')

                # Imprimir el resultado
                print(f'🛈 Número de clase: {clase}, Nombre de la fruta: [red]{nombre_fruta}[red]')

                start_point, end_point = (int(b[0]),int(b[1])), (int(b[2]),int(b[3]))
                w, h = end_point[0]-start_point[0],end_point[1]-start_point[1]
                center=[int(start_point[0]+w/2),int(start_point[1]+h/2)]

                # Pero las coordenadas del centro van desde la esquina superior izquierda, si queremos desde el centro de la cámara:
                real_center=[center[0]-frame_center[0],center[1]-frame_center[1]]
                confs = box.conf.item()
    
                # Guardamos información de las cajas (x,y,w,h,(cx,cy) desde la esquina superior izquierda,(cx,cy) desde el centro de la imagen)
                boxes_data=[start_point[0],start_point[1],w,h,center,real_center,confs,nombre_fruta] 
                boxes_list.append(boxes_data)

        return boxes_list

    def publish_image(self):
        if self.m_img != Image():
            self.pub_image.publish(self.m_img) 

    # Hilo
    def timer_callback(self):
        if self.last_image != Image():

            # Guardar imagen BGR
            bgr = np.frombuffer(self.last_image.data, dtype=np.uint8).reshape(self.last_image.height, self.last_image.width, -1)
            img_bgr = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            # Procesar la imagen
            results = self.model(img_bgr)
            boxes_list= self._cajas_yolo(img_bgr,results)
            boxes_list = self.f_FILTRO_margen(boxes_list,img_bgr) # Realizamos un filtro para eliminar las cajas que toquen las esquinas de la imagen de la cámara
            boxes_list = self.f_FILTROS(boxes_list,img_bgr) # Realizamos un filtro para eliminar las cajas que toquen las esquinas de la imagen de la cámara
            boxes_list = self.f_FILTRO_CONF(boxes_list,0.8) # Realizamos un filtro para eliminar las cajas con la confianza baja
            img_bgr_anclaje = img_bgr.copy()

            masks_list=self.segment_roi(boxes_list,results,img_bgr_anclaje)
            if boxes_list:
                self.dibujar_cajas(boxes_list,img_bgr)
                if masks_list:
                    oriented_bounding_boxes,oriented_img = self.get_oriented_bb(boxes_list,img_bgr_anclaje,masks_list)
                    puntos_medios = self.calcular_pts_medios(oriented_bounding_boxes,oriented_img)
                    puntos_anclaje_list,bgr_puntos = self.puntos_anclaje(oriented_bounding_boxes,img_bgr_anclaje,puntos_medios,masks_list)

                    self.fruits = Fruits()


                    if len(puntos_anclaje_list) == len(boxes_list):
                        print('[green]✅ Deteccicon correcta. Puntos de anclaje correctos.')
                                        
                        for i in range(0,len(puntos_anclaje_list)):
                            start_point = puntos_anclaje_list[i][0]
                            end_point = puntos_anclaje_list[i][1]
                            centroid = puntos_anclaje_list[i][2]
                            nombre_fruta = boxes_list[i][7]
                            angulo = self.calcular_angulo(start_point,end_point) 


                            if nombre_fruta=="Green Apple":
                                self.fruits.green_apple=centroid
                                self.fruits.angles[0]=angulo
                            elif nombre_fruta=="Red Apple":
                                self.fruits.red_apple=centroid
                                self.fruits.angles[1]=angulo
                            elif nombre_fruta=="Banana":
                                self.fruits.banana=centroid
                                self.fruits.angles[2]=angulo
                            elif nombre_fruta=="Lemon":
                                self.fruits.lemon=centroid
                                self.fruits.angles[3]=angulo
                            elif nombre_fruta=="Orange":
                                self.fruits.orange=centroid
                                self.fruits.angles[4]=angulo
                            elif nombre_fruta=="Peach":
                                self.fruits.peach=centroid
                                self.fruits.angles[5]=angulo
                            elif nombre_fruta=="Pear":
                                self.fruits.pear=centroid
                                self.fruits.angles[6]=angulo
                            elif nombre_fruta=="Pepper":
                                self.fruits.pepper=centroid
                                self.fruits.angles[7]=angulo
                            elif nombre_fruta=="Kiwi":
                                self.fruits.kiwi=centroid
                                self.fruits.angles[8]=angulo

                            print(f'\n[bold green_yellow]✅ Fruta: {nombre_fruta} ✅ \nAnclaje 1: {start_point} \nAnclaje 2: {end_point} \nCentroide: {centroid} \nAngulo rad: {angulo}')

                        # Publish
                        self.pub_fruits.publish(self.fruits)

                # Publicar imagen procesada
                img_bgr = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2BGR)
                self.m_img.data = img_bgr.reshape(-1).tolist() 
                self.m_img.header.stamp = self.get_clock().now().to_msg() # Indica cuando se genera el mensaje
                # self.pub_image.publish(self.m_img)  
    
def main(args=None):
    rclpy.init(args=args)

    config = get_mongo_config()

    camera_node = DetectFruits(config)

    try:
        rclpy.spin(camera_node)  
    except KeyboardInterrupt:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()