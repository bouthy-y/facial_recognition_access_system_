# facial_recognition_access_system_

## Description
Ce projet propose un système de contrôle d'accès innovant basé sur la reconnaissance faciale. Utilisant un Raspberry Pi et divers composants électroniques, le système garantit un contrôle sécurisé et automatisé des accès.

## Fonctionnalités principales
- Détection des visages avec la bibliothèque `face_recognition`.
- Détection de mouvement à l'aide d'un capteur PIR.
- Contrôle d'un servo-moteur pour l'ouverture et la fermeture d'une porte.
- Indicateurs lumineux (LED verte et rouge) pour signaler l'état d'accès.
- Affichage des messages d'état sur un écran LCD 16x2.

## Matériel requis
- Raspberry Pi 4
- Capteur PIR
- Servo-moteur
- LEDs (verte et rouge)
- Écran LCD 16x2 avec interface I2C
- Caméra compatible Raspberry Pi
- Câblage et alimentation nécessaires

## Dépendances
Les bibliothèques suivantes sont utilisées :
- `face_recognition`
- `cv2` (OpenCV)
- `numpy`
- `RPi.GPIO`
- `smbus2`
  

Assurez-vous d'installer les dépendances dans un environnement virtuel avec la commande :
```bash
pip install face_recognition opencv-python-headless numpy smbus2





