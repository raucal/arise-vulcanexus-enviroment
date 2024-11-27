import snap7
import DB_connection
from rich import print
import time

class ClaseGuia:
    def __init__(self, config):

        print('[yellow]----- Guia Lineal JXCP1 -----[/yellow]')
        self.config = config
        self.guide_connection = False

        # 1. PLC connection definition  
        IP =  self.config.get('plc').get('ip_address') 
        RACK = self.config.get('plc').get('rack') 
        SLOT = self.config.get('plc').get('slot')

        # Numero de la DB de lectura
        self.DB_READ_NUMBER = self.config.get('plc').get('DB_read_number') 
        self.START_ADDRESS_READ = self.config.get('plc').get('start_address_read')    # Inicio 
        self.SIZE_READ = self.config.get('plc').get('size_read')    # Tamaño  

        # Numero de la DB de control (start)
        self.OUTPUTS_NUMBER = self.config.get('plc').get('DB_outputs_number') 
        self.START_ADDRESS_OUTPUTS = self.config.get('plc').get('start_address_read')     # Inicio 
        self.SIZE_OUTPUTS = self.config.get('plc').get('size_outputs')    # Tamaño   

        # Numero de la DB (Move to positions 1 and 2)
        self.DB_NUMBER_AUTO = self.config.get('plc').get('DB_move_number')
        self.START_ADDRESS_AUTO = self.config.get('plc').get('start_address_move')    # Inicio 
        self.SIZE_AUTO = self.config.get('plc').get('size_move')   # Tamaño  

        # Parámetros
        self.speed =  self.config.get('plc').get('speed')
        self.position_1 = self.config.get('plc').get('position_1')
        self.position_2 = self.config.get('plc').get('position_2')
        
        # 1.1 PLC connection via snap7 client module
        self.plc = snap7.client.Client()
        print("[blue]Estableciendo conexión con guía lineal...")
        try:
            self.plc.connect(IP, RACK, SLOT)
            self.guide_connection = True
            print("[green]Conexión establecida con gúia lineal")
        except Exception as e:
            print(F"[red]<ClaseGuia>[ERROR] -> {e}")
            print("[red]No se ha podido establecer la conexión con la guia lineal")

    def lectura_db(self, show = False):

        # 2. Point DB and read data
        self.db_r = self.plc.db_read(self.DB_READ_NUMBER, self.START_ADDRESS_READ, self.SIZE_READ)

        # print('[yellow]----- JXCP1_Inputs -----[/yellow]')

        step_data_out = DB_connection.read_usint(self.db_r ,0)
        state_bits = DB_connection.read_usint(self.db_r ,1)
        ready_flag = DB_connection.read_bool(self.db_r ,2,0)
        current_position = DB_connection.read_real(self.db_r ,4)
        current_speed = DB_connection.read_int(self.db_r ,8)
        pushing_force = DB_connection.read_int(self.db_r ,10)
        target_position = DB_connection.read_real(self.db_r ,12)
        alarm_1 = DB_connection.read_usint(self.db_r ,16)
        alarm_2 = DB_connection.read_usint(self.db_r ,17)
        alarm_3 = DB_connection.read_usint(self.db_r ,18)
        alarm_4 = DB_connection.read_usint(self.db_r ,19)
        ready_control = DB_connection.read_bool(self.db_r, 21,1)

        if show: 
            print(f'STEP DATA OUT: {step_data_out}')
            print(f'STATE BITS: {state_bits}')
            print(f'READY FALG: {ready_flag}')
            print(f'CURRENT POSITION: {current_position}')
            print(f'CURRENT SPEED: {current_speed}')
            print(f'PUSHING FORCE: {pushing_force}')
            print(f'TARGET POSITION: {target_position}')
            print(f'ALARM_1: {alarm_1}')
            print(f'ALARM_2: {alarm_2}')
            print(f'ALARM_3: {alarm_3}')
            print(f'ALARM_4: {alarm_4}')
            print(f'READY_CONTROL: {ready_control}')

        return step_data_out, state_bits, ready_flag, current_position, current_speed, pushing_force, alarm_1, alarm_2, alarm_3, alarm_4, ready_control


    def start(self):
        print('[yellow]----- JXCP1_Outputs -----[/yellow]')
        response = None

        try:
            # 2. Point DB and read data
            db_control = self.plc.db_read(self.OUTPUTS_NUMBER, self.START_ADDRESS_OUTPUTS, self.SIZE_OUTPUTS)
            self.db_r = self.plc.db_read(self.DB_READ_NUMBER, self.START_ADDRESS_READ, self.SIZE_READ)

            """Comprobamos que el motor esta encendido"""
            print('[blue]Comprobando que el motor está encendido...')
            svon = DB_connection.read_bool(db_control,6,6)
            # print('SVON: ',svon)
            if svon == False:
                print("[red]<ClaseGuia>/start/ Motor no encendido. Arrancando motor...")
                DB_connection.write_bool(self.plc, self.OUTPUTS_NUMBER, db_control,6,6,True)
            else:
                print("[green]<ClaseGuia>/start/ Motor correctamente encendido!")
            
            """Comprobamos que el control remoto esté habilitado"""
            ready_control = DB_connection.read_bool(self.db_r, 21,1)
            if not ready_control: 
                response= False
                message = "Desactiva la seta de emergencia y presiona el botón Reload Safety Relay"
                print("[red]<claseGuía>/start/ Desactiva la seta de emergencia y presiona el botón Reload Safety Relay")
            
            else:
                """Comprobamos que la velocidad fijada es mayor que 15"""
                # print('[blue]Comprobamos que la velocidad fijada es mayor que 15"')
                set_speed = DB_connection.read_int(db_control,8)
                
                if set_speed < 30:
                    # print('[bold yellow]Warning. Velocidad de JXCP1 baja')
                    DB_connection.write_int(self.plc, self.OUTPUTS_NUMBER, db_control, 8, self.speed)
                    set_speed = DB_connection.read_int(db_control,8)
                print(f'[blue]<ClaseGuia>/start/ Velocidad fijada a: {set_speed}')

                """Fijamos las posiciones de la guia"""
                print('[yellow]-----  SET modo auto.  -----[/yellow]')
                # 2. Point DB and read data
                db_auto = self.plc.db_read(self.DB_NUMBER_AUTO, self.START_ADDRESS_AUTO, self.SIZE_AUTO)

                    # POSE 1
                self.pose_1 = DB_connection.read_real(db_auto,2)

                DB_connection.write_real(self.plc, self.DB_NUMBER_AUTO, db_auto, 2, self.position_1)
                self.pose_1 = DB_connection.read_real(db_auto,2)
                print('SET pose_1: ',self.pose_1)

                    # POSE 2
                self.pose_2 = DB_connection.read_real(db_auto,6)

                DB_connection.write_real(self.plc, self.DB_NUMBER_AUTO, db_auto, 6, self.position_2)
                self.pose_2 = DB_connection.read_real(db_auto,6)
                print('SET pose_2: ',self.pose_2)
                
                response = True
                message = ""
                
        except Exception as e:
            print(f"[red]<start> ERROR -> {e}")
            response = False
            message = e

        return response, message

    def move_to_1(self):
        try:
            # print('[yellow]-----  MOVE 1 -----[/yellow]')

            # 2. Point DB and read data
            db_auto = self.plc.db_read(self.DB_NUMBER_AUTO,self.START_ADDRESS_AUTO,self.SIZE_AUTO)

            move_to_1 = DB_connection.read_bool(db_auto,0,0)
            move_to_2 = DB_connection.read_bool(db_auto,0,1)   

            # print(f'move to 1: {move_to_1} | move to 2: {move_to_2}')
            self.current_position = DB_connection.read_real(self.db_r,4)

                # movemos a la posicion 1
            if self.current_position != self.pose_1:
                DB_connection.write_bool(self.plc,self.DB_NUMBER_AUTO,db_auto,0,0,True)

            DB_connection.write_bool(self.plc,self.DB_NUMBER_AUTO,db_auto,0,0,False)
            move_to_1 = DB_connection.read_bool(db_auto,0,0)
            # print('move to 1: ',move_to_1)
            return True, ""
        except Exception as e:
            return False, e


    def move_to_2(self):
        try:
            # print('[yellow]-----  MOVE 2 -----[/yellow]')
            self.db_r = self.plc.db_read(self.DB_READ_NUMBER, self.START_ADDRESS_READ, self.SIZE_READ)

            # 2. Point DB and read data
            db_auto = self.plc.db_read(self.DB_NUMBER_AUTO,self.START_ADDRESS_AUTO,self.SIZE_AUTO)

            move_to_1 = DB_connection.read_bool(db_auto,0,0)
            move_to_2 = DB_connection.read_bool(db_auto,0,1)        

            # print(f'move to 1: {move_to_1} | move to 2: {move_to_2}')
            self.current_position = DB_connection.read_real(self.db_r,4)

                # movemos a la posicion 2
            if self.current_position != self.pose_2:
                DB_connection.write_bool(self.plc,self.DB_NUMBER_AUTO,db_auto,0,1,True)

            DB_connection.write_bool(self.plc,self.DB_NUMBER_AUTO,db_auto,0,1,False)
            move_to_2 = DB_connection.read_bool(db_auto,0,1)
            # print('move to 2: ',move_to_2)
            return True, ""
        except Exception as e:
            return False, e
