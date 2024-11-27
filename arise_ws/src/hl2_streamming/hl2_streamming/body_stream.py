import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Pose
from sensor_msgs.msg import JointState
from pynput import keyboard

from . import hl2ss
from . import hl2ss_lnm
import threading
import time

# Nodo de ROS2 para recibir datos espaciales desde HoloLens y publicarlos en tópicos
class HoloLensSpatialInputNode(Node):
    def __init__(self):
        super().__init__('body_stream')
        
        # Dirección de HoloLens
        self.host = "192.168.101.91"
        self.enable = True
        
        self.head_pose = None
        self.eye_pose = None
        self.right_hand_pose = None
        self.left_hand_pose = None

        self.start_connection = False
        
        # Suscriptores de ROS2
        self.create_subscription(String, 'battery_disassembly/hololens/qr_coordinates', self.QRCallback, 1) 
        
        # Publicadores de ROS2
        self.head_pose_publisher = self.create_publisher(Pose, 'hl2_streamming/head_pose', 10)
        self.eye_ray_publisher = self.create_publisher(Vector3, 'hl2_streamming/eye_ray', 10)
        self.hand_left_publisher = self.create_publisher(Pose, 'hl2_streamming/left_hand_pose', 10)
        self.hand_right_publisher = self.create_publisher(Pose, 'hl2_streamming/right_hand_pose', 10)

        print("Esperando códigos QR para establecer")
        self.thread = threading.Thread(target = self.wait4connection)
        self.thread.start()
    
    def wait4connection(self):
        retry = 0
        while True:
            if self.start_connection:
                try:
                    print("Estbleciendo conexión...")
                    # Cliente para recibir datos de entrada espacial desde HoloLens
                    self.client = hl2ss_lnm.rx_si(self.host, hl2ss.StreamPort.SPATIAL_INPUT)
                    self.client.open()
                    
                    # Listener para capturar la tecla 'esc' y detener la ejecución
                    listener = keyboard.Listener(on_press=self.on_press)
                    listener.start()
                    break
                except Exception as e:
                    if retry<3:
                        print("Error en la conexión con hololens")
                        retry+=1
                        time.sleep(10)
                    else:
                        break

               

        # Timer de ROS2 para obtener datos y publicarlos en un bucle a 30 Hz
        self.timer = self.create_timer(0.01, self.spatial_input_callback)
        self.publish_timer = self.create_timer(0.2, self.publish_messages)
        

    
    def QRCallback(self, msg):
        self.start_connection = True
        print("recibido")
        
    def on_press(self, key):
        if key == keyboard.Key.esc:
            self.enable = False
            self.timer.cancel()
            self.publish_timer.cancel()
            self.client.close()
            rclpy.shutdown()

    def spatial_input_callback(self):
        if not self.enable:
            return
        
        # Recibe el siguiente paquete de datos espaciales
        data = self.client.get_next_packet()
        si = hl2ss.unpack_si(data.payload)

        self.get_logger().info(f'Tracking status at time {data.timestamp}')

        # Publicar la pose de la cabeza
        if si.is_valid_head_pose():
            head_pose = si.get_head_pose()
            pose_msg = Pose()
            pose_msg.position.x = float(head_pose.position[0])
            pose_msg.position.y = float(head_pose.position[1])
            pose_msg.position.z = float(head_pose.position[2])
            # Aquí podrías convertir forward y up a una orientación (quaternion)
            pose_msg.orientation.x = float(head_pose.forward[0])
            pose_msg.orientation.y = float(head_pose.forward[1])
            pose_msg.orientation.z = float(head_pose.forward[2])
            pose_msg.orientation.w = 1.0  # Esto es un placeholder, deberías calcular el quaternion.
            self.head_pose = pose_msg
            
            self.get_logger().info(f'Head pose: Position={head_pose.position}')
        else:
            self.get_logger().info('No head pose data')

        # Publicar eye ray
        if si.is_valid_eye_ray():
            eye_ray = si.get_eye_ray()
            eye_ray_msg = Vector3()
            eye_ray_msg.x = float(eye_ray.direction[0])
            eye_ray_msg.y = float(eye_ray.direction[1])
            eye_ray_msg.z = float(eye_ray.direction[2])
            self.eye_pose = eye_ray_msg
            
            self.get_logger().info(f'Eye ray: Direction={eye_ray.direction}')
        else:
            self.get_logger().info('No eye tracking data')

        # Publicar pose de la mano izquierda
        if si.is_valid_hand_left():
            hand_left = si.get_hand_left()
            pose = hand_left.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
            pose_msg = Pose()
            pose_msg.position.x = float(pose.position[0])
            pose_msg.position.y = float(pose.position[1])
            pose_msg.position.z = float(pose.position[2])
            # Deberías hacer una conversión a quaternion para la orientación
            pose_msg.orientation.x = float(pose.orientation[0])
            pose_msg.orientation.y = float(pose.orientation[1])
            pose_msg.orientation.z = float(pose.orientation[2])
            pose_msg.orientation.w = float(pose.orientation[3])
            self.left_hand_pose = pose_msg
            
            self.get_logger().info(f'Left wrist pose: Position={pose.position}')
        else:
            self.get_logger().info('No left hand data')

        # Publicar pose de la mano derecha
        if si.is_valid_hand_right():
            hand_right = si.get_hand_right()
            pose = hand_right.get_joint_pose(hl2ss.SI_HandJointKind.Wrist)
            pose_msg = Pose()
            pose_msg.position.x = float(pose.position[0])
            pose_msg.position.y = float(pose.position[1])
            pose_msg.position.z = float(pose.position[2])
            # Deberías hacer una conversión a quaternion para la orientación
            pose_msg.orientation.x = float(pose.orientation[0])
            pose_msg.orientation.y = float(pose.orientation[1])
            pose_msg.orientation.z = float(pose.orientation[2])
            pose_msg.orientation.w = float(pose.orientation[3])
            self.right_hand_pose = pose_msg
            
            self.get_logger().info(f'Right wrist pose: Position={pose.position}')
        else:
            self.get_logger().info('No right hand data')
    
    def publish_messages(self):
        if self.head_pose != None:    self.head_pose_publisher.publish(self.head_pose)
        if self.eye_pose != None:  self.eye_ray_publisher.publish(self.eye_pose)
        if self.right_hand_pose != None:  self.hand_left_publisher.publish(self.right_hand_pose)
        if self.left_hand_pose != None:   self.hand_right_publisher.publish(self.left_hand_pose)


def main(args=None):
    rclpy.init(args=args)
    node = HoloLensSpatialInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
