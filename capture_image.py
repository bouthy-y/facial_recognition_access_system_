import cv2
import os
from time import sleep

# Configuration
OUTPUT_DIR = "known_faces"  # Dossier où sauvegarder les images
CAMERA_INDEX = 0            # 0 pour la caméra par défaut
IMAGE_COUNT = 20            # Nombre d'images à capturer par personne
DELAY_BETWEEN_SHOTS = 0.5   # Délai entre les captures en secondes
RESOLUTION = (640, 480)     # Résolution de la caméra

# Créer le dossier de sortie s'il n'existe pas
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# Démarrer la capture vidéo
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])

# Charger le classificateur de visage Haar Cascade
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def capture_faces(name):
    """
    Capture plusieurs images d'un visage et les enregistre dans le dossier known_faces
    """
    print(f"\nPrêt à capturer {IMAGE_COUNT} images pour {name}. Appuyez sur 's' pour commencer...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erreur de lecture de la caméra")
            return
        
        # Afficher le flux vidéo
        cv2.imshow('Capture Visage - Appuyez sur S pour commencer', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # Commencer la capture
            break
        elif key == ord('q'):  # Quitter
            return False
    
    print("Début de la capture... Sourirez! :)")
    sleep(1)  # Petit délai avant de commencer
    
    count = 0
    while count < IMAGE_COUNT:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Convertir en niveaux de gris pour la détection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Détecter les visages
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(100, 100)
        
        # Si un visage est détecté
        if len(faces) == 1:
            (x, y, w, h) = faces[0]
            
            # Dessiner un rectangle autour du visage
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Extraire le visage (avec une marge)
            margin = 20
            face_img = frame[y-margin:y+h+margin, x-margin:x+w+margin]
            
            # Enregistrer l'image si elle a la bonne taille
            if face_img.size > 0:
                count += 1
                filename = f"{OUTPUT_DIR}/{name}_{count}.jpg"
                cv2.imwrite(filename, face_img)
                print(f"Image {count}/{IMAGE_COUNT} sauvegardée: {filename}")
                
                # Afficher un compte à rebours
                cv2.putText(frame, f"Capture: {count}/{IMAGE_COUNT}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Attendre avant la prochaine capture
                sleep(DELAY_BETWEEN_SHOTS)
        
        # Afficher le flux vidéo
        cv2.imshow('Capture Visage - Appuyez sur S pour commencer', frame)
        
        # Quitter avec la touche 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False
    
    return True

def main():
    print("=== Programme de Capture de Visages ===")
    print("Ce script va capturer des images pour la reconnaissance faciale.")
    print("Les images seront sauvegardées dans le dossier 'known_faces'.")
    
    while True:
        name = input("\nEntrez le nom de la personne (ou 'q' pour quitter): ").strip()
        if name.lower() == 'q':
            break
        
        if not name:
            print("Veuillez entrer un nom valide")
            continue
        
        success = capture_faces(name)
        if not success:
            break
    
    # Nettoyage
    cap.release()
    cv2.destroyAllWindows()
    print("\nProgramme terminé. Vous pouvez maintenant utiliser ces images pour la reconnaissance faciale.")

if _name_ == "_main_":
    main()
