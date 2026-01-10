#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# dieses Programm ist die Maturaarbeit von: Modjtaba Taib
# Schule: ISME St.Gallen
# Betreuer: Herr André Nuber
# Datum: 07.01.2026
# Dieses Programm erfordert LEGO EV3 MicroPython v2.0 oder höher.
# Es sind vier Sensoren im Einsatz: zwei Infrarotsensor, ein Ultraschallsensor und ein Farbsensor.
# Der Farbsensor und der Infrarotsensor sind auf der linken Seite des Roboters montiert.
# Der Ultraschallsensor ist auf der Vorderseite des Roboters montiert.
# Der Ultraschallsensor dient dazu, Hindernisse zu erkennen und ihnen auszuweichen.
# Der Infrarotsensoren dienen dazu, eine Linie zu verfolgen.
# Der Farbsensor dient dazu, beim Eintreffen einer Linkskurve die Markierung am Boden zu erkennen und entsprechend zu reagieren.
# Der Roboter fährt vorwärts, bis er ein Hindernis erkennt. Dann biegt er nach rechts.
# Wenn der Roboter auf eine Linkskurve trifft, erkennt der Farbsensor die Linie und der Roboter biegt nach links ab.
# Somit kann der Roboter seinen Weg aus dem Labyrinth finden.
#lenkung ohne pausen
# Wandabstand mit infrarotsensor regeln


ev3 = EV3Brick()

# Motoren & Fahrwerk
motor_links = Motor(Port.B)
motor_rechts = Motor(Port.A)
fahrwerk = DriveBase(motor_links, motor_rechts, 43.2, 104.0)

# infrarotsensoren links
ir_vorne = InfraredSensor(Port.S4)
ir_hinten = InfraredSensor(Port.S1)
ultraschall = UltrasonicSensor(Port.S2)
licht_links = ColorSensor(Port.S3)


# Parameter
Soll_Abstand = 19        # mm
Toleranz = 1             # Totzone ±1 mm
Geschwindigkeit = 40
Abstand = ultraschall.distance()
Abstandsgrenze = 60
Helligkeitsgrenze_links = 25
Abstand_v = ir_vorne.distance() 
Abstand_h = ir_hinten.distance()
Korr_Faktor = Abstand_v + Abstand_h / 2
Max_Abstand = 22        # maximal sinnvoller Wert
K_Par = -1.5                   # Prarallelitätsfaktor
K_Abs = -2.2                  # Abstandsfaktor
#Hellikkeitsgrenze_schwarz = 9
#Hellikkeitsgrenze_blau = 11



while True:

    Abstand_V = ir_vorne.distance() 
    Abstand_h = ir_hinten.distance()
    Abstand = ultraschall.distance()
    Helligkeit_links = licht_links.reflection() 
    
    Parallelitaet = Abstand_V - Abstand_h
    Mittelwert = (Abstand_V + Abstand_h) / 2
    Abstands_Fehler = Mittelwert - Soll_Abstand

    ev3.screen.clear()
    ev3.screen.print("Vorne:  {} mm".format(Abstand_V))
    ev3.screen.print("Hinten: {} mm".format(Abstand_h))

    # --- Wand-Erkennung ---
    
    wand_rechts_da = Abstand_V < 25 and Abstand_h < 25
    keine_wand_rechts = Abstand_V >= 25 and Abstand_h >= 25
    
    if wand_rechts_da:
        # Normale Lenkregelung
        
        if abs(Abstands_Fehler) <= Toleranz and  abs(Parallelitaet) <= Toleranz:
            Lenkung = 0
        else:
            Lenkung = Parallelitaet * K_Par + Abstands_Fehler * K_Abs
    else:
        # Keine Wand → keine Lenkung
        Lenkung = 0
        
    #if abs(Abstands_Fehler) <= Toleranz and  abs(Parallelitaet) <= Toleranz:
        #Lenkung = 0
        #elif abs(Abstands_Fehler) > Max_Abstand:
        #Lenkung = 0
    #else:
        #Lenkung = Parallelitaet * K_Par + Abstands_Fehler * K_Abs
    
    # --- Hindernisse / Farben / Abbiegen ---
    if Abstand < Abstandsgrenze and not wand_rechts_da:
        fahrwerk.stop()
        wait(300)
        fahrwerk.turn(-90)
        wait(300)
        continue   # ← WICHTIG: nicht weiterfahren!

    elif (Abstand < Abstandsgrenze or Helligkeit_links < 10) and wand_rechts_da:
        fahrwerk.stop()
        wait(300)
        fahrwerk.turn(90)
        wait(300)
        continue

    if not wand_rechts_da:

    # Schwarz → rechts abbiegen
        if Helligkeit_links <= 10:
            fahrwerk.stop()
            wait(300)
            fahrwerk.turn(-90)
            wait(300)
            continue

    if wand_rechts_da:
        if 10 < Helligkeit_links <= 12:
            fahrwerk.stop()
            wait(300)
            fahrwerk.turn(90)
            wait(300)
            continue
    # --- Fahrwerk nur hier starten ---
    fahrwerk.drive(Geschwindigkeit, Lenkung)
    print(Lenkung)
    wait(500)





