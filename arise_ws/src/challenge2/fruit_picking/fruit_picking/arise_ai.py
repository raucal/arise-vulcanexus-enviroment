# == IMPORTACIONES ==================================================
# > Transcriptor VOSK
from vosk import Model, KaldiRecognizer
import pyaudio

# > Utilidades
from fruit_picking import get_mongo_config, user
import threading # Para realizar hilos
from fruit_picking.ollama_request import ollama_talker

# > ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rich import print


class AriseAI(Node):

    def __init__(self,config):
        super().__init__('arise_ai')
        print('-- ARISE AI --')
        
        self.config = config
        self.model_language =self.config['language_model']

        print(f"[yellow]Cargando modelo de lenguaje {self.model_language}...")
        if self.model_language == 'en':
            model = Model(f'/home/{user}/'+'vosk-models/vosk-model-en-us')
        elif self.model_language == 'en_light':
            model = Model(f'/home/{user}/'+'vosk-models/vosk-model-small-en-us')
        elif self.model_language == 'es':
            model = Model(f'/home/{user}/'+'vosk-models/vosk-model-es')
        elif self.model_language == 'es_light':
            model = Model(f'/home/{user}/'+'vosk-models/vosk-model-small-es')

        else:
            raise Exception('Lenguaje del modelo seleccionado no válido.')

        self.set_publishers_and_subscribers()    
            
        # model = Model("/home/guicab@cartif.local/ros2_ws/src/speech_recon/speech_recon/vosk-model-small-es-0.42")
        # model = Model("vosk-model-small-es-0.42")

        # model = Model("/home/guicab@cartif.local/ros2_ws/src/speech_recon/speech_recon/vosk-model-es-0.42")
        self.recognizer = KaldiRecognizer(model, 16000)

        mic = pyaudio.PyAudio()
        self.stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
        self.stream.start_stream()
        # record_thread = threading.Thread(target=self.record, args=())
        # record_thread.start()
        
        self.ollama = ollama_talker('arise_2') # Inicializamos Servidor Ollama
        
        
        self.record() # Inicializamos grabación

    def set_publishers_and_subscribers(self):
        self.publisher_ = self.create_publisher(String, '/fruit_picking/ollama/selected_fruit', 10) # Creamos el publicador

    def record(self):
        print('Starting recording ...')
        while True:
            data = self.stream.read(4096)
            if self.recognizer.AcceptWaveform(data):
                text = self.recognizer.Result()
                print(f"'{text[14:-3]}'")

                if text[14:-3] != '':
                    self.send_transcription(text[14:-3])
                    # ollama_thread = threading.Thread(target=self.send_transcription, args=(text[14:-3],))
                    # ollama_thread.start()

    def send_transcription(self, prompt):
        msg = String()
        answer = self.ollama.talk(prompt)
        print(f'{answer} | {len(answer)} | {type(answer)}')
        if not "NONE" in answer: # Si el modelo devuelve NONE, es decir que no ha recibido nada, no publicamos
            msg.data = answer
            self.publisher_.publish(msg)
            self.get_logger().info(f'Incoming request: {prompt}\nCreated answer: {answer}')
            # self.get_logger().info(f'Publishing: {msg.data}')
                

def main(args=None):
    rclpy.init(args=args)
    config = get_mongo_config()
    arise_ai= AriseAI(config)

    rclpy.spin(arise_ai)
    arise_ai.destroy_node()
    rclpy.shutdown()