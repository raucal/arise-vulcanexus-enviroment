#!/usr/bin/env python3
'''Este script se utiliza en caso de que se esté ejecutando en Linux nativo, no en un contenedor'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from rich import print
from gtts import gTTS
import tempfile
import os
from playsound import playsound

class AudioInterface(Node):
    def __init__(self):
        super().__init__('audio_interface') # Si no salta el error -> AttributeError: 'AudioInterface' object has no attribute '_default_callback_group'
        self.set_publishers_and_subscribers()
    
    def set_publishers_and_subscribers(self):
        self.create_subscription(String, 'audio_interface/play_audio', self.textCallback, 1)      

    def textCallback(self, data):
        message = data.data
        # Reproducir el mensaje en audio
        try:
            # Pyttsx3
            print(f"[blue]Reproduciendo el mensaje: {message}")

            ''' Pyttsx3 '''
            # engine = pyttsx3.init()
            # voices = engine.getProperty('voices')
            # engine.setProperty('voice', voices[0].id)
            # engine.setProperty('rate', 150)     # setting up new voice rate
            # # engine.setProperty('volume',100)  
            # engine.say(message) 
            # engine.runAndWait()

            ''' Gttts y PlaySound'''
            tts = gTTS(text=message, lang='es')
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as fp:
                temp_file_path = fp.name
                tts.save(temp_file_path)

            # Reproducir el archivo de audio con playsound
            playsound(temp_file_path)

            # Eliminar el archivo temporal después de la reproducción
            os.remove(temp_file_path)

            print(f"[green]Mensaje reproducido")
        except Exception as e:
            print(f"[red]Error al reproducir el mensaje:{message}")
            print(f"[red]{e}")

def main(args=None):
    rclpy.init(args=args)

    audio_node = AudioInterface()
    audio_node.textCallback(String(data='Audio inicializado: Probando, probando'))

    try:
        rclpy.spin(audio_node)  
    finally:
        audio_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
