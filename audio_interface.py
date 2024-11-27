''' El objetivo del script es darle voz al robot'''
import roslibpy
import pyttsx3
from rich import print

class AudioInterface():
    def __init__(self):
        self.client = roslibpy.Ros(host='192.168.107.38', port=9090)  #same as rosbridge port
        self.client.run()
        self.set_publishers_and_subscribers()
    
    def set_publishers_and_subscribers(self):
        self.text_to_audio_topic = roslibpy.Topic(self.client, 'cartif/my_robot_interfaces/play_audio', 'std_msgs/String')
        self.text_to_audio_topic.subscribe(self.textCallback)

    def textCallback(self, data):
        message = data['data']
        # Reproducir el mensaje en audio
        try:
            print(f"[blue]Reproduciendo el mensaje: {message}")
            engine = pyttsx3.init()
            voices = engine.getProperty('voices')
            engine.setProperty('voice', voices[0].id)
            engine.setProperty('rate', 150)     # setting up new voice rate
            engine.setProperty('volume',100.0)  
            engine.say(message) 
            engine.runAndWait()
            print(f"[green]Mensaje reproducido")
        except Exception as e:
            print(f"[red]Error al reproducir el mensaje:{message}")
            print(f"[red]{e}")

def main():
    audio_node = AudioInterface()
    audio_node.textCallback({'data': 'Audio inicializado: Probando, probando'})
    try:
        while True:
            pass
    except KeyboardInterrupt:
        audio_node.close_connection()

if __name__ == '__main__':
    main()