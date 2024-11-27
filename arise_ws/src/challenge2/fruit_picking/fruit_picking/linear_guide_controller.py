import rclpy
from rclpy.node import Node
from fruit_picking import get_mongo_config, claseGuia
from my_robot_interfaces.msg import JXCP1Inputs
from example_interfaces.srv import Trigger
from rich import print

class LinearGuideController(Node):
    def __init__(self, config, linear_guide):
        super().__init__('guide_node')
        self.config = config
        self.JXCP1 = linear_guide
        self.timer_period = self.config["plc"]["timer_period"]

        # Iniciar comunicadores
        self.set_publishers_and_subscribers()
        self.set_servers()
        self.timer = self.create_timer(self.timer_period, self.read_db)
        
        print("[green]>>Nodo guía inicializado")

    def set_publishers_and_subscribers(self):
        self.pub_db_data = self.create_publisher(JXCP1Inputs, 'linear_guide/current_info',10)

    def set_servers(self):
        self.create_service(Trigger, 'linear_guide/move_to_position_1', self.move_to_position_1)
        self.create_service(Trigger, 'linear_guide/move_to_position_2', self.move_to_position_2)

    def read_db(self):
        step_data_out, state_bits, ready_flag, current_position, current_speed, pushing_force, alarm_1, alarm_2, alarm_3, alarm_4, ready_control = self.JXCP1.lectura_db()
        current_info = JXCP1Inputs()
        current_info.step_data_out = step_data_out
        current_info.state_bits = state_bits
        current_info.ready_flag = ready_flag
        current_info.current_position = current_position
        current_info.current_speed = current_speed
        current_info.current_force = pushing_force
        current_info.alarm_1 = alarm_1
        current_info.alarm_2 = alarm_2
        current_info.alarm_3 = alarm_3
        current_info.alarm_4 = alarm_4
        current_info.ready_control = ready_control

        self.pub_db_data.publish(current_info)

    def move_to_position_1(self, request, response):
        print(f"[orange1]<move_to_position_1> Moviendo guía a posición 1")
        response.success, response.message = self.JXCP1.move_to_1()
        return response
    
    def move_to_position_2(self, request, response):
        print(f"[orange1]<move_to_position_2> Moviendo guía a posición 2")
        response.success, response.message=  self.JXCP1.move_to_2()
        return response
 
def main(args=None):
    rclpy.init(args=args)

    config = get_mongo_config()

    try:
        linear_guide = claseGuia.ClaseGuia(config)
        response, msg = linear_guide.start()
        if linear_guide.guide_connection and response:
            try:
                linear_guide.move_to_2()
                guide_node = LinearGuideController(config, linear_guide)
                rclpy.spin(guide_node)
            except KeyboardInterrupt:
                guide_node.destroy_node()
                rclpy.shutdown()
        else:
            rclpy.shutdown()
    except Exception as e:
        print(f"[red][ERROR] -> {e}")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
