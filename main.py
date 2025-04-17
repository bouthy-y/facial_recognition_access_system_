import face_recognition
import cv2
import numpy as np
import os
import RPi.GPIO as GPIO
import time
import smbus2
from smbus2 import SMBus

# Configuration des GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuration du PIR
PIR_PIN = 4  # GPIO4 pour le capteur PIR
GPIO.setup(PIR_PIN, GPIO.IN)

# Configuration du servo-moteur
SERVO_PIN = 17
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz = 20ms
pwm.start(0)  # Position initiale à 0°

# Configuration des LEDs
LED_VERTE = 22  # GPIO22 pour LED verte (accès autorisé)
LED_ROUGE = 27  # GPIO27 pour LED rouge (accès refusé)
GPIO.setup(LED_VERTE, GPIO.OUT)
GPIO.setup(LED_ROUGE, GPIO.OUT)

# Configuration LCD 16x2 I2C
I2C_ADDR = 0x27  # Adresse I2C typique pour LCD
LCD_WIDTH = 16    # Largeur du LCD en caractères

# Commandes LCD
LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80  # Adresse 1ère ligne
LCD_LINE_2 = 0xC0  # Adresse 2ème ligne
LCD_BACKLIGHT = 0x08  # Rétroéclairage activé

# Timing
E_PULSE = 0.0005
E_DELAY = 0.0005

# Initialisation du bus I2C
try:
    bus = smbus2.SMBus(1)  # Raspberry Pi version 2 utilise 1
except:
    bus = smbus2.SMBus(0)  # Raspberry Pi version 1 utilise 0

def lcd_init():
    """Initialise l'écran LCD"""
    lcd_byte(0x33, LCD_CMD)  # Initialisation
    lcd_byte(0x32, LCD_CMD)  # Initialisation
    lcd_byte(0x06, LCD_CMD)  # Sens du curseur
    lcd_byte(0x0C, LCD_CMD)  # Display On,Cursor Off, Blink Off
    lcd_byte(0x28, LCD_CMD)  # 4 bits, 2 lignes, 5x8 dots
    lcd_byte(0x01, LCD_CMD)  # Effacer l'écran
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    """Envoie un byte au LCD"""
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    """Toggle enable"""
    time.sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | 0x04))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR, (bits & ~0x04))
    time.sleep(E_DELAY)

def lcd_string(message, line):
    """Affiche une chaîne sur le LCD"""
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

def set_servo_angle(angle):
    duty = angle / 18 + 2  # Conversion angle -> duty cycle
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Temps pour atteindre la position
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)  # Évite les vibrations

def control_leds(acces_autorise):
    """Contrôle les LEDs en fonction de l'accès"""
    if acces_autorise:
        GPIO.output(LED_VERTE, GPIO.HIGH)
        GPIO.output(LED_ROUGE, GPIO.LOW)
    else:
        GPIO.output(LED_VERTE, GPIO.LOW)
        GPIO.output(LED_ROUGE, GPIO.HIGH)

# Initialisation LCD
lcd_init()
lcd_string("Systeme pret", LCD_LINE_1)
lcd_string("En attente...", LCD_LINE_2)

# Chemin vers le répertoire des images autorisées
KNOWN_FACES_DIR = "known_faces"

# Charger les visages connus
print("Chargement des visages connus...")
known_face_encodings = []
known_face_names = []

for name in os.listdir(KNOWN_FACES_DIR):
    image_path = os.path.join(KNOWN_FACES_DIR, name)
    try:
        image = face_recognition.load_image_file(image_path)
        encodings = face_recognition.face_encodings(image)
        
        if len(encodings) > 0:
            known_face_encodings.append(encodings[0])
            known_face_names.append(os.path.splitext(name)[0])
            print(f"Visage chargé: {name}")
        else:
            print(f"Aucun visage détecté dans {name} - Vérifiez la qualité de l'image")
    except Exception as e:
        print(f"Erreur avec {name}: {str(e)}")

print(f"{len(known_face_names)} visages valides chargés")

if not known_face_encodings:
    print("Aucun visage valide trouvé. Vérifiez votre dossier known_faces.")
    exit()

# Initialiser la caméra
video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Variables pour la détection
process_this_frame = True
servo_active = False
face_detection_active = False
pir_timeout = 5  # Temps en secondes sans détection PIR avant désactivation
last_pir_detection = 0

print("En attente de détection PIR...")

try:
    while True:
        # Vérifier l'état du PIR
        pir_state = GPIO.input(PIR_PIN)
        
        if pir_state:
            last_pir_detection = time.time()
            if not face_detection_active:
                print("Personne détectée par PIR - Activation reconnaissance faciale")
                face_detection_active = True
                lcd_string("Detection en", LCD_LINE_1)
                lcd_string("cours...", LCD_LINE_2)
        
        # Désactiver la détection faciale si timeout PIR
        if face_detection_active and (time.time() - last_pir_detection > pir_timeout):
            print("Timeout PIR - Retour à l'état initial")
            face_detection_active = False
            if servo_active:
                set_servo_angle(0)  # Fermer la porte
                servo_active = False
            control_leds(False)
            lcd_string("Systeme pret", LCD_LINE_1)
            lcd_string("En attente...", LCD_LINE_2)
            continue
        
        # Si personne détectée par PIR, faire la reconnaissance faciale
        if face_detection_active:
            ret, frame = video_capture.read()
            if not ret:
                print("Erreur de capture vidéo")
                break
            
            # Redimensionner et convertir l'image
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
            rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            
            if process_this_frame:
                # Détection des visages
                face_locations = face_recognition.face_locations(rgb_small_frame)
                name = "Inconnu"  # Par défaut, accès refusé
                
                if face_locations:
                    # Encodage des visages détectés
                    face_encodings = face_recognition.face_encodings(
                        rgb_small_frame, 
                        face_locations,
                        num_jitters=1
                    )
                    
                    for face_encoding in face_encodings:
                        # Comparaison avec les visages connus
                        matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=0.6)
                        
                        # Trouver la meilleure correspondance
                        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                        best_match_index = np.argmin(face_distances)
                        
                        if matches[best_match_index]:
                            name = known_face_names[best_match_index]
                            if not servo_active:
                                print(f"Personne autorisée détectée: {name}")
                                set_servo_angle(90)  # Rotation à 90°
                                control_leds(True)   # Allumer LED verte
                                lcd_string("Bienvenue", LCD_LINE_1)
                                lcd_string(name[:16], LCD_LINE_2)  # Tronquer si trop long
                                servo_active = True
                        else:
                            print("Personne non autorisée détectée")
                            control_leds(False)  # Allumer LED rouge
                            lcd_string("Acces refuse", LCD_LINE_1)
                            lcd_string("Non autorise", LCD_LINE_2)
            
            process_this_frame = not process_this_frame
            
            # Affichage des résultats
            for (top, right, bottom, left), display_name in zip(face_locations, [name]*len(face_locations)):
                top *= 4; right *= 4; bottom *= 4; left *= 4
                color = (0, 255, 0) if display_name != "Inconnu" else (0, 0, 255)
                cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
                cv2.putText(frame, display_name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)
            
            cv2.imshow('Reconnaissance Faciale', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Nettoyage à la fin du programme
    set_servo_angle(0)  # Remise à zéro
    pwm.stop()
    GPIO.output(LED_VERTE, GPIO.LOW)
    GPIO.output(LED_ROUGE, GPIO.LOW)
    lcd_byte(0x01, LCD_CMD)  # Effacer l'écran
    lcd_string("Systeme arrete", LCD_LINE_1)
    GPIO.cleanup()
    video_capture.release()
    cv2.destroyAllWindows()
    print("Programme arrêté")
