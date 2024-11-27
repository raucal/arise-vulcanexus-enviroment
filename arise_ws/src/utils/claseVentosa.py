import logging
import time
from pymodbus.client.sync_diag import ModbusTcpClient as ModbusClient
from rich import print

logger = logging.getLogger('claseVentosa')

class VG():
    def __init__(self, config):
        
        self.config = config
        self.ip= self.config.get('ventosa').get('ip') # IP
        self.port= self.config.get('ventosa').get('port')


        self.client = ModbusClient(
            self.ip,
            port=self.port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        # self.open_connection()
    
        logger.info('Clase de la ventosa iniciada')
        print(f'[bold sky_blue2]IP de la ventosa {self.ip}\nPuerto: {self.port}')

    def hex2percent(self,percent):
        ''' Transform a percentage to the hex number needed for the gripper.'''
        num=int((255*percent)/100)
        hex_value=num
        return hex_value

    def action_order(self,string):
        if string=='On' or string=='on':
            hex_str = 0x0100
        elif string=='Off' or string=='off':
            hex_str = 0x0000          
        else:    
            hex_str= 0x0200
        return hex_str

    def reverseByteOrder(self,data):
        #"Reverses the byte order of an int (16-bit) or long (32-bit) value"
        # Courtesy Vishal Sapre
        wordCount = len(hex(data)[2:].replace('L','')[::4])
        val = 0
        for i in range(wordCount):
            val = (val << 8) | (data & 0xff)
            data >>= 8
            val = (val << 8) | (data & 0xff)
            data >>= 8
        return val

#== Configuraciçon ================================================================================================

    def open_connection(self):
        """Opens the connection with a gripper."""
        print(f"[white]Estableciendo conexión con ventosa con IP: {self.ip}")
        try:
            self.client.connect()
            logger.debug('Conexión establecida con ventosa')
            print("[green]>>Conexión establecida con ventosa")
            return True,None
        except Exception as e:
            logger.error('Conexión no establecida con ventosa')
            print("[red]>>Conexión no establecida con ventosa")
            return False, str(e)


    def close_connection(self):
        """Closes the connection with the gripper."""
        self.client.close()

    def get_vacuum_limit(self):
        """Sets and reads the current limit.
        The limit is provided and must be given in mA (milli-amperes).
        The limit is 500mA per default and should never be set above 1000 mA.
        """
        result = self.client.read_holding_registers(
            address=2, count=1, unit=65)
        limit_mA = result.registers[0]
        return limit_mA

#== Recogida de datos ================================================================================================
    def get_channelA_vacuum(self):
        """Reads the actual vacuum on Channel A.
        The vacuum is provided in 1/1000 of relative vacuum.
        Please note that this differs from the setpoint given in percent,
        as extra accuracy is desirable on the actual vacuum.
        """
        result = self.client.read_holding_registers(
            address=258, count=1, unit=65)

        # print('--------------------')
        # # print('Revision de errores:')
        # print(f'Contenido de result: {result}, tipo de contenido {type(result)}')
        # # print(f'\n Qué tiene dentro: {dir(result)}')
        # print(f'\n args: {result.args}\n fcode: {result.fcode}\n isError: {result.isError}\n message: {result.message}\n string: {result.string}\n with_traceback: {result.with_traceback}')
        # print(f'Contenido de registers: {result.registers}, tipo de contenido {type(result.registers)}')
        # print(f'\n [Para [0]] Contenido de registers: {result.registers[0]}, tipo de contenido {type(result.registers[0])}')

        try:
            vacuum = result.registers[0]
            return vacuum
        except AttributeError:
            # print('[ERROR] No ha sido posible recibir datos de las ventosas.')
            return None

    def get_channelB_vacuum(self):
        """Same as the one of channel A."""
        result = self.client.read_holding_registers(
            address=259, count=1, unit=65)
        try:
            vacuum = result.registers[0]
            return vacuum
        except AttributeError:
            # print('[ERROR] No ha sido posible recibir datos de las ventosas.')
            return None
    def get_channelC_vacuum(self):
        """Same as the one of channel A."""
        result = self.client.read_holding_registers(
            address=260, count=1, unit=65)
        try:
            vacuum = result.registers[0]
            return vacuum
        except AttributeError:
            # print('[ERROR] No ha sido posible recibir datos de las ventosas.')
            return None
    def get_channelD_vacuum(self):
        """Same as the one of channel A."""
        result = self.client.read_holding_registers(
            address=261, count=1, unit=65)
        try:
            vacuum = result.registers[0]
            return vacuum
        except AttributeError:
            # print('[ERROR] No ha sido posible recibir datos de las ventosas.')
            return None
#%% Set channel control ================================================================================================
    def set_channelA_control(self, modename, command):
        """This register allows for control of channel A.

        The register is split into two 8-bit fields:
        Bits 15-8        Bits 7-0
        Control mode     Target vacuum

        The Control mode field must contain one of these three values:

        Value    Name    Description
        0 (0x00) Release Commands the channel to release
                            any work item and stop the pump,
                            if not required by the other channel.
        1 (0x01) Grip    Commands the channel to build up
                            and maintain vacuum on this channel.
        2 (0x02) Idle    Commands the channel to neither release nor grip.
                            Workpieces may "stick" to the channel
                            if physically pressed towards its vacuum cups,
                            but the VG will use slightly less power.

            The Target vacuum field sets the level of vacuum
            to be build up and maintained by the chann el.
            It is used only when the control mode is 1 (0x01) / Grip.
            The target vacuum should be provided in % vacuum.
            It should never exceed 80.

            Examples:
            Setting the register value 0 (0x0000)
                will command the VG to release the work item.
            Setting the register value 276 (0x0114)
                will command the VG to grip at 20 % vacuum.
            Setting the register value 296 (0x0128)
                will command the VG to grip at 40 % vacuum.
            Setting the register value 331 (0x014B)
                will command the VG to grip at 75 % vacuum.
            Setting the register value 512 (0x0200)
                will command the VG to idle the channel.
        """
        if modename == "Release":
            modeval = 0x0000
        elif modename == "Grip":
            modeval = 0x0100
        elif modename == "Idle":
            modeval = 0x0200
        result = self.client.write_register(
            address=0, value=modeval+command, unit=65)

    def set_channelB_control(self, modename, command):
        """Same as the one of channel A."""
        if modename == "Release":
            modeval = 0x0000
        elif modename == "Grip":
            modeval = 0x0100
        elif modename == "Idle":
            modeval = 0x0200
        result = self.client.write_register(
            address=1, value=modeval+command, unit=65)
    def set_channelC_control(self, modename, command):
        """Same as the one of channel A."""
        if modename == "Release":
            modeval = 0x0000
        elif modename == "Grip":
            modeval = 0x0100
        elif modename == "Idle":
            modeval = 0x0200
        result = self.client.write_register(
            address=2, value=modeval+command, unit=65)
    def set_channelD_control(self, modename, command):
        """Same as the one of channel A."""
        if modename == "Release":
            modeval = 0x0000
        elif modename == "Grip":
            modeval = 0x0100
        elif modename == "Idle":
            modeval = 0x0200
        result = self.client.write_register(
            address=3, value=modeval+command, unit=65)
            
# == Activamos / desactivamos todas las ventosas ================================================================================================
    def vacuum_on(self, sleep_sec=1.0):
        """Turns on all vacuums."""
        modeval = 0x0100  # grip
        command = 0x00ff  # 100 % vacuum
        commands = [modeval+command, modeval+command,modeval+command, modeval+command]
        result = self.client.write_registers(
            address=0, values=commands, unit=65)

        print("Turn on all vacuums.")
        start = time.time()
        # while True:
        #     print("Current vacuums, channel A: " +
        #           str(self.get_channelA_vacuum()) +
        #           ", channel B: " +
        #           str(self.get_channelB_vacuum()))
        #     if time.time() - start > sleep_sec:
        #         break

    def release_vacuum(self):
        """Releases all vacuums"""
        modeval = 0x0000  # release
        command = 0x0000  # 0 % vacuum
        commands = [modeval+command, modeval+command,modeval+command, modeval+command]

        print("\nRelease all vacuums.")
        result = self.client.write_registers(
            address=0, values=commands, unit=65)
        time.sleep(1.0)

# == Activamos / desactivamos las filas de ventosas independientemente ================================================================================================

#----------------- ACTIVATE -----------------

    def vacuum_on_channelA(self, sleep_sec=1.0):
        """Turns on the vacuum of channel A."""
        modeval = 0x100  # grip
        command = 0xff  # 100 % vacuum
        print(f'\n -- \n Action: {modeval} | Percent {command} ')
        print(f'\n Action: {type(modeval)} | Percent {type(command)}\n -- \n ')
        result = self.client.write_register(
            address=0, value=modeval+command, unit=65)

        print("\nTurn on the vacuum of channel A.")
        start = time.time()
        while True:
            print("Current channel A's vacuum: " +
                  str(self.get_channelA_vacuum()))
            if time.time() - start > sleep_sec:
                break

    def vacuum_on_channelB(self, sleep_sec=1.0):
        """Turns on the vacuum of channel B."""
        modeval = 0x0100  # grip
        command = 0x00ff  # 100 % vacuum

        result = self.client.write_register(
            address=1, value=modeval+command, unit=65)

        print("\nTurn on the vacuum of channel B.")
        start = time.time()
        while True:
            print("Current channel B's vacuum: " +
                  str(self.get_channelB_vacuum()))
            if time.time() - start > sleep_sec:
                break

    def vacuum_on_channelC(self, sleep_sec=1.0):
        """Turns on the vacuum of channel C."""
        modeval = 0x0100  # grip
        command = 0x00ff  # 100 % vacuum
        result = self.client.write_register(
            address=2, value=modeval+command, unit=65)

        print("\nTurn on the vacuum of channel C.")
        start = time.time()
        while True:
            print("Current channel C's vacuum: " +
                  str(self.get_channelC_vacuum()))
            if time.time() - start > sleep_sec:
                break

    def vacuum_on_channelD(self, sleep_sec=1.0):
        """Turns on the vacuum of channel D."""
        modeval = 0x0100  # grip
        command = 0x00ff  # 100 % vacuum
        result = self.client.write_register(
            address=3, value=modeval+command, unit=65)

        print("\nTurn on the vacuum of channel D.")
        start = time.time()
        while True:
            print("Current channel D's vacuum: " +
                  str(self.get_channelD_vacuum()))
            if time.time() - start > sleep_sec:
                break

# ----------------- RELEASE -----------------

    def release_vacuum_channelA(self):
        """Releases the vacuum of channel A."""
        modeval = 0x0000  # release
        command = 0x0000  # 0 % vacuum
        print("\nRelease the vacuum of channel A.")
        result = self.client.write_register(
            address=0, value=modeval+command, unit=65)
        time.sleep(1.0)

    def release_vacuum_channelB(self):
        """Releases the vacuum of channel B."""
        modeval = 0x0000  # release
        command = 0x0000  # 0 % vacuum
        print("\nRelease the vacuum of channel B.")
        result = self.client.write_register(
            address=1, value=modeval+command, unit=65)
        time.sleep(1.0)
    def release_vacuum_channelC(self):
        """Releases the vacuum of channel B."""
        modeval = 0x0000  # release
        command = 0x0000  # 0 % vacuum
        print("\nRelease the vacuum of channel B.")
        result = self.client.write_register(
            address=2, value=modeval+command, unit=65)
        time.sleep(1.0)
    def release_vacuum_channelD(self):
        """Releases the vacuum of channel B."""
        modeval = 0x0000  # release
        command = 0x0000  # 0 % vacuum
        print("\nRelease the vacuum of channel B.")
        result = self.client.write_register(
            address=3, value=modeval+command, unit=65)
        time.sleep(1.0)
        
#== Si queremos activar según porcentajes ================================================================================================
    def vacuum_channelA(self,modeval,command):
        """Controls the vacuum of channel A."""
        result = self.client.write_register(
            address=0, value=modeval+command, unit=65)
        time.sleep(1.0)
    
    def vacuum_channelB(self,modeval,command):
        """Controls the vacuum of channel B."""
        result = self.client.write_register(
            address=1, value=modeval+command, unit=65)
        time.sleep(1.0)
    def vacuum_channelC(self,modeval,command):
        """Controls the vacuum of channel C."""
        result = self.client.write_register(
            address=2, value=modeval+command, unit=65)
        time.sleep(1.0)
    def vacuum_channelD(self,modeval,command):
        """Controls the vacuum of channel D."""
        result = self.client.write_register(
            address=3, value=modeval+command, unit=65)
        time.sleep(1.0)

    def vacuum_control(self,commands, sleep_sec=1.0):
        # modeval = 0x0100  # grip
        # command = 0x00ff  # 100 % vacuum
        # commands = [modeval+command, modeval+command,modeval+command, modeval+command]
        result = self.client.write_registers(
            address=0, values=commands, unit=65)

    # def onrobotVGP20(self,channel='',action='Off',percent=0):
    #     action=action_order(action)
    #     percent=hex2percent(percent)
    #     for letter in [*channel]:
    #         print(f'Canal {letter}')
    #         if letter == 'A':
    #             self.vacuum_channelA(action,percent)
    #         elif letter == 'B':
    #             self.vacuum_channelB(action,percent)
    #         elif letter == 'C':
    #             self.vacuum_channelC(action,percent)
    #         elif letter == 'D':
    #             self.vacuum_channelD(action,percent)

    def onrobotVGP20(self,channel='ABCD',action='Off',percent=0,config=[0,0,0,0]):
        '''
        Función de control de la garra de ventosas VGP20, de cuatro canales.
        IMPORTANTE: Es necesario llamar una vez a esta función vacía (sin argumentos) para inicializar la lista de configuraciones.

        Parameters
        ----------
        channel : str, optional
            Lista de caracteres que representan los canales de la ventosa sobre los que se quiere actuar. Por defecto selecciona todos 'ABCD'.
        action : str, optional
            Indicador de si queremos activar o desactivar la fila de ventosas [On/Off]. Por defecto 'Off'.
        percent : int, optional
            Porcentaje que queremos aplicar a las ventosas [0-100%]. Por defecto 0%.
        config : list, optional
            Lista de configuraciones previa de las ventosas. Necesario para mantener el estado previo de la ventosa cuando se realice un cambio. Por defecto [0,0,0,0].

        Returns
        -------
        ordenes : list
            Devuelve la lista actualizada d ela configuración de las ventosas.

        '''
        #action_A,percent_A,action_B,percent_B,action_C,percent_C,action_D,percent_D=[0,0,0,0,0,0,0,0]
        action=self.action_order(action)
        percent=self.hex2percent(percent)
        if action ==0:
            percent=0
        channel_A=config [0]
        channel_B=config [1]
        channel_C=config [2]
        channel_D=config [3]

        for letter in [*channel]:
            # print(f'Definiendo canal {letter} a {round(percent*100/255)} %')
            if letter == 'A':
                channel_A=action+percent
            elif letter == 'B':
                channel_B=action+percent
            elif letter == 'C':
                channel_C=action+percent
            elif letter == 'D':
                channel_D=action+percent
        ordenes=[channel_A,channel_B,channel_C,channel_D]
        
        self.vacuum_control(ordenes)
        return ordenes


#== Lectura de datos ================================================================================================

    def test_lectura(self):
        reg = self.client.read_holding_registers(address=256, count=6, slave=65)
        return reg
