import cv2
import pyautogui  # Librería para obtener dimensiones de pantalla
import time
from rich import print

def reproducir_video(url):
    # Obtener las dimensiones de la pantalla automáticamente
    ancho_monitor, alto_monitor = pyautogui.size()

    # Crear la ventana antes del bucle y establecer propiedades para pantalla completa
    cv2.namedWindow('video', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('video', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    connection_established = False  # Variable para rastrear el estado de la conexión

    while True:
        cap = cv2.VideoCapture(url)

        # Verificar si la conexión se estableció correctamente
        if not cap.isOpened():
            if connection_established:
                print("Conexión perdida. Intentando reconectar en 3 segundos...")
                connection_established = False
            else:
                print("No se pudo abrir el flujo de video. Intentando reconectar en 3 segundos...")
            time.sleep(3)
            continue  # Intentar reconectar
        else:
            if not connection_established:
                print("[green]Conexión establecida exitosamente.")
                connection_established = True

        # Mantener el programa en ejecución mientras el video se reproduce
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # Escalar el frame a las dimensiones del monitor
                frame_escalado = cv2.resize(frame, (ancho_monitor, alto_monitor))

                # Mostrar el frame en pantalla completa
                cv2.imshow('video', frame_escalado)

                # Esperar a que se presione la tecla 'q' para salir del programa
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cap.release()
                    cv2.destroyAllWindows()
                    return  # Salir de la función y terminar el programa
            else:
                print("Conexión interrumpida. Intentando reconectar en 3 segundos...")
                cap.release()
                connection_established = False  # Restablecer el estado de conexión
                time.sleep(3)
                break  # Salir del bucle interno para intentar reconectar

video_url = "https://Cervera5R:Cervera5R47151@192.168.101.91/api/holographic/stream/live.mp4?holo=true&pv=true&mic=true&loopback=true&RenderFromCamera=true"

# Llamar a la función para reproducir el video con OpenCV
reproducir_video(video_url)
