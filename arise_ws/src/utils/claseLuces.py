import socket
from rich import print

class TableLights:
    def __init__(self, ip, show=None):
        self.ip = ip
        self.color = None
        self.show = show
        self.lights_connected = self.gateway_connection()
    
    def gateway_connection(self):
        try:
            target_ip=(self.ip, 10034)
            print(f"[blue]->Estableciendo conexión con {target_ip[0]}")
            print("[green]Conexión establecida")
            self.sk=socket.socket()
            self.sk.connect(target_ip)
            return True
        except Exception as e:
            print(f"[red][ERROR] -> {e}")
            return False
    
    def change_color(self, color):
        if self.lights_connected:
            try:
                self.color = color
                if self.show:
                    print("Cambiando de color a", self.color)
                self.sk.send(self.color.encode())
                return True
            except Exception as e:
                print(f"[red][ERROR] -> {e}")
                return False
        else:
            print("[red]Petición rechazada. Establece conexión con el gateway")
    
    def get_last_color(self):
        return self.color
    
    def gateway_disconnection(self):
        try:
            self.change_color("[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]")
            self.sk.close()
        except Exception as e:
            print(f"[red][ERROR] -> {e}")
            return False
        
    
