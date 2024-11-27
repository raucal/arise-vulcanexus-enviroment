import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from battery_disassembly import get_mongo_config, claseRobot, clasePinza
import time
from rich import print

class UR10eInterface(Node):
    def __init__(self, config, robot, gripper):
        super().__init__('ur10e_screw')
        self.config = config
        self.robot = robot
        self.gripper = gripper
        self.screw_pose = self.config["ur10e"]["screw_pose"]
        self.box_pose = self.config["ur10e"]["box_pose"]
        self.screws_in_box = 0

        # Iniciar Comunicadores
        self.set_publishers_and_subscribers()

    def set_publishers_and_subscribers(self):
        self.create_subscription(String, 'battery_disassembly/robot/send_screw_to_box', self.placeCallback, 1)

    def placeCallback(self, data):
        print(f"<placeCallback> Mensaje recibido: {data.data}")
        if data.data == "Send To Box":
            print("[yellow]<placeCallback> Dejando tornillo en la caja de destino...")

            try:
                ''' Cerrar garra '''
                self.gripper.move_and_wait_for_pos(255, 255, 255)
                time.sleep(2)

                ''' Alejar '''
                pose1 = self.robot.poseTrans(self.robot.getForwardKinematics(),[0,0.05,0,0,0,0])
                self.robot.moveL(pose1)

                ''' Dejar tornillo en la caja'''
                self.robot.moveJ_IK(self.box_pose)
                self.gripper.move_and_wait_for_pos(0, 255, 255) # Angle, Force, Speed

                ''' Volver a la posición de recogida'''
                self.robot.moveJ_IK(self.screw_pose)

                self.screws_in_box +=1
            except Exception as e:
                print(f"[red][ERROR]-> Error al conectar con la pinza: {e}")
        else:
            print("[orange1]<placeCallback> No hay programada ninguna acción con este mensaje")

def main(args = None):
    rclpy.init(args=args)

    config = get_mongo_config()

    try:
        robot = claseRobot.Robot(ip = config["ur10e"]["ip_address"], mode = config["ur10e"]["mode"])
        if robot.robot_connected:
            gripper = clasePinza.RobotiqGripper(ip = config["ur10e"]["ip_address"])
            ur10e_node = UR10eInterface(config, robot, gripper)

            gripper.activate()
            gripper.move_and_wait_for_pos(0, 255, 255)
            # Mover robot a posición de inicio
            robot.moveJ_IK(ur10e_node.screw_pose)

            try:
                rclpy.spin(ur10e_node)  
            finally:
                robot.control_disconnection()
                robot.receive_disconnection()
                gripper.disconnect()

                ur10e_node.destroy_node()
                rclpy.shutdown()
    except Exception as e:
        print(f"[red][ERROR]->{e}")

if __name__ == '__main__':
    main()
    

