#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


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

"""
#lenkung ohne pausen
# Wandabstand mit infrarotsensor regeln
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, InfraredSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

ev3 = EV3Brick()

# Motoren & Fahrwerk
motor_links = Motor(Port.B)
motor_rechts = Motor(Port.A)
fahrwerk = DriveBase(motor_links, motor_rechts, 43.2, 104.0)

# infrarotsensoren links
ir_vorne = InfraredSensor(Port.S4)
ir_hinten = InfraredSensor(Port.S1)

# Parameter
SOLL_ABSTAND = 18        # mm
TOLERANZ = 1             # Totzone ±1 mm
GESCHWINDIGKEIT = 40
abstand_V = ir_vorne.distance() - 3
abstand_h = ir_hinten.distance()
#MAX_LENKUNG = 3     # maximaler Lenkwinkel
KORR_FAKTOR = abstand_V + abstand_h / 2
#MIN_ABSTAND = 15          # minimal messbarer Abstand
#MAX_ABSTAND = 25        # maximal sinnvoller Wert
K_Par = 2                   # Prarallelitätsfaktor
K_Abs = 0.5                  # Abstandsfaktor

while True:
    abstand_V = ir_vorne.distance() - 3
    abstand_h = ir_hinten.distance()

    Parallelitaet = (abstand_V) - abstand_h 
    mittelwert = (abstand_V + abstand_h) / 2
    fehler = mittelwert - SOLL_ABSTAND
    
    # Totzone: kleine Abweichungen ignorieren
    if abs(fehler) <= TOLERANZ and  abs(Parallelitaet) <= TOLERANZ:
        lenkung = 0
    else:
        # Proportionale Korrektur
        lenkung = Parallelitaet * K_Par + fehler * K_Abs  

    # Geschwindigkeit konstant, Lenkung gleichzeitig
    fahrwerk.drive(GESCHWINDIGKEIT, lenkung)

    # Optional: Abstand auf Display anzeigen
    ev3.screen.clear()
    #ev3.screen.print("Abstand: {} mm".format(abstand))

    wait(50)
    
"""    






"""
#diesen Teil umfasst Linie folgen mit Infrarot, Sackgasse erkennen mit Ultraschal und
#Rechtskurven fahren mit der Meldung des Farbsensors
#alle Schritte sind funktionsfähig, jedoch ohne maximale Abstand des IfSensors. 
#lenkung ohne pausen
# Wandabstand mit infrarotsensor regeln
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, InfraredSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

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
SOLL_ABSTAND = 18        # mm
TOLERANZ = 1             # Totzone ±1 mm
GESCHWINDIGKEIT = 40
abstand = ultraschall.distance()
abstandsgrenze = 50
helligkeitsgrenze_links = 25
abstand_V = ir_vorne.distance() - 3
abstand_h = ir_hinten.distance()
#MAX_LENKUNG = 3     # maximaler Lenkwinkel
KORR_FAKTOR = abstand_V + abstand_h / 2
#MIN_ABSTAND = 15          # minimal messbarer Abstand
#MAX_ABSTAND = 25        # maximal sinnvoller Wert
K_Par = 2                   # Prarallelitätsfaktor
K_Abs = 0.5                  # Abstandsfaktor



while True:
    abstand_V = ir_vorne.distance() - 3
    abstand_h = ir_hinten.distance()
    abstand = ultraschall.distance()
    helligkeit_links = licht_links.reflection() 
    
    Parallelitaet = (abstand_V) - abstand_h 
    mittelwert = (abstand_V + abstand_h) / 2
    fehler = mittelwert - SOLL_ABSTAND
    
    # Totzone: kleine Abweichungen ignorieren
    if abs(fehler) <= TOLERANZ and  abs(Parallelitaet) <= TOLERANZ:
        lenkung = 0
    else:
        # Proportionale Korrektur
        lenkung = Parallelitaet * K_Par + fehler * K_Abs 
        
    if abstand < abstandsgrenze:
        fahrwerk.stop()
        wait(300)
        fahrwerk.turn(-90)
        wait(300) 
    elif abstand > 30 and helligkeit_links < helligkeitsgrenze_links:
            fahrwerk.stop()  # anhalten
            wait(500)
            fahrwerk.turn(90)      # nach rechts drehen
            wait(500)    

    # Geschwindigkeit konstant, Lenkung gleichzeitig
    fahrwerk.drive(GESCHWINDIGKEIT, lenkung)

    # Optional: Abstand auf Display anzeigen
    ev3.screen.clear()
    #ev3.screen.print("Abstand: {} mm".format(abstand))

    wait(50)

"""





#lenkung ohne pausen
# Wandabstand mit infrarotsensor regeln
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, InfraredSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

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

    wait(50)






"""
while True:
    abstand_V = ir_vorne.distance() 
    abstand_h = ir_hinten.distance()
    abstand = ultraschall.distance()
    helligkeit_links = licht_links.reflection() 
    
    Parallelitaet = (abstand_V) - abstand_h 
    mittelwert = (abstand_V + abstand_h) / 2
    fehler = mittelwert - SOLL_ABSTAND
    ev3.screen.clear()
    ev3.screen.print("Vorne:  {} mm".format(abstand_V))
    ev3.screen.print("Hinten: {} mm".format(abstand_h))
    # Totzone: kleine Abweichungen ignorieren
    
    if abs(fehler) <= TOLERANZ and  abs(Parallelitaet) <= TOLERANZ:
        lenkung = 0
    elif abs(fehler) > MAX_ABSTAND:
        lenkung = 0
    else:
        # Proportionale Korrektur
        lenkung = Parallelitaet * K_Par + fehler * K_Abs 
    
    
    wand_rechts_da = abstand_V < 30 and abstand_h < 30

    if wand_rechts_da:
    # normale Regelung
        lenkung = Parallelitaet * K_Par + fehler * K_Abs
    else:
    # Wand verloren → nicht korrigieren
        lenkung = 0
        
    if abstand < abstandsgrenze and fehler > MAX_ABSTAND:
            fahrwerk.stop()
            wait(300)
            fahrwerk.turn(-90)
            wait(300) 
    elif abstand < abstandsgrenze and fehler < MAX_ABSTAND:
            fahrwerk.stop()
            wait(300)
            fahrwerk.turn(90)
            wait(300)
    elif helligkeit_links <= 10 and fehler < MAX_ABSTAND:
            fahrwerk.stop()  # anhalten
            wait(500)
            fahrwerk.turn(90)      # nach rechts drehen
            wait(500)    
    elif 10 < helligkeit_links <= 12 and fehler > MAX_ABSTAND:
            fahrwerk.stop()  # anhalten
            wait(500)
            fahrwerk.turn(-90)      # nach rechts drehen
            wait(500)  
    # Geschwindigkeit konstant, Lenkung gleichzeitig
    fahrwerk.drive(GESCHWINDIGKEIT, lenkung)

    wait(500)

"""





    

"""   
    
# Hindernis ausweichen mit Ultraschallsensor
while True:
    abstand = ultraschall.distance()
    helligkeit_links = licht_links.reflection() 
    if abstand < abstandsgrenze:
        fahrwerk.stop()
        wait(300)
        fahrwerk.turn(-90)
        wait(300) 
    elif abstand > 30 and helligkeit_links < helligkeitsgrenze_links:
            fahrwerk.stop()  # anhalten
            wait(500)
            fahrwerk.turn(90)      # nach rechts drehen
            wait(500)
    fahrwerk.drive(40, 0)
    wait (50)

    
    
    






ev3 = EV3Brick()

motor_links = Motor(Port.A)
motor_rechts = Motor(Port.B)

fahrwerk = DriveBase(
    motor_links,
    motor_rechts,
    43.200,   # Raddurchmesser in mm
    104.00    # Radabstand in mm
)

licht_links = ColorSensor(Port.S3)
ir = InfraredSensor(Port.S4)          # IR jetzt vorne
ultraschall = UltrasonicSensor(Port.S2)  # Ultraschall jetzt seitlich

# Grenzwerte
helligkeitsgrenze_links = 20
#ir.mode = "IR-PROX"
ir_grenze = 2            # gewünschter IR-Abstand vorne
abstandsgrenze = 40       # gewünschter seitlicher Abstand (Ultraschall)

# Hauptschleife
while True:
    ir_vorne = ir.distance()                 # Hindernis vorne
    abstand_seite = ultraschall.distance()  # Abstand seitlich
    helligkeit_links = licht_links.reflection()

    # 🔴 Hindernis vorne → ausweichen
    if ir_vorne < ir_grenze:
        fahrwerk.stop()
        wait(300)
        fahrwerk.turn(-90)
        wait(300)

    # 🟡 Linie links + genug Seitenabstand → nach rechts drehen
    elif abstand_seite > abstandsgrenze and helligkeit_links < helligkeitsgrenze_links:
        fahrwerk.stop()
        wait(500)
        fahrwerk.turn(90)
        wait(500)

    # 🟢 Seitenabstand regeln (Wand folgen)
    else:
        fehler = abstandsgrenze - abstand_seite
        lenkung = fehler * -2
        fahrwerk.drive(50, lenkung)
        wait(50)
"""

"""
ev3 = EV3Brick()
motor_links = Motor(Port.A)                                      #Motoren und Sensoren werden difiniert
motor_rechts = Motor(Port.B)
fahrwerk = DriveBase(motor_links, motor_rechts, 43.200, 104.00)  # Motor links, Motor rechts, Raddurchmesser in mm (damit kann die Geschwindigkeit angepasst werden.), Radabstand in mm
licht_links = ColorSensor(Port.S3)
ir = InfraredSensor(Port.S4) # Infrarotsensor wird difiniert
ultraschall = UltrasonicSensor(Port.S2)


helligkeitsgrenze_links = 20             # Zahlen anpassen
ir.mode = "IR-PROX"                      # Zahlen anpassen
abstandsgrenze = 40                      # Zahlen anpassen
ir_grenze = 2               # IR-Wert (0–100) → 20 = nah

# Hindernis ausweichen mit Ultraschallsensor
while True:
    abstand = ultraschall.distance() 
    abstand_links = ir.distance()
    helligkeit_links = licht_links.reflection()
    
    if abstand < abstandsgrenze:
        fahrwerk.stop()
        wait(300)
        fahrwerk.turn(-90)
        wait(300)
        abstand = ultraschall.distance() 
    elif abstand_links > 3 and helligkeit_links < helligkeitsgrenze_links:
            fahrwerk.stop()  # anhalten
            wait(500)
            fahrwerk.turn(90)      # nach rechts drehen
            wait(500)
    else:
         fehler= ir_grenze - abstand_links
         lenkung = fehler * -2
         fahrwerk.drive(50, lenkung)
         wait (50)
"""        

"""
# ---------- IR-Sensor testen ----------
ev3.speaker.beep()          #ev3 biebst 
fahrwerk.drive(100, 0)      # vorwärts fahren

# Wert einmal lesen
wert = ir.distance()        #Infrarotsensor wird in Nahemodus eingesetzt. 

#eine Wand verfolgen nur mit IR-Sensor
while True:
    # Wert anzeigen
    ev3.screen.clear()
    wert = ir.distance() 
    ev3.screen.print("IR:", wert)   # zeigt IR-Wert 0–100 auf dem Bildschrirm des Steines
    fehler= ir_grenze - wert
    lenkung = fehler * -2
    fahrwerk.drive(100, lenkung)
    wait (50)
    

# Hindernis ausweichen mit Ultraschallsensor
while True:
    abstand = ultraschall.distance() 
    if abstand < abstandsgrenze:
        fahrwerk.stop()
        wait(300)
        fahrwerk.drive(-20,0)
        wait(300)
        fahrwerk.turn(90)
        wait(300)
        abstand = ultraschall.distance() 
        if abstand < abstandsgrenze:
            fahrwerk.stop()
            fahrwerk.turn(-180)
            wait(300)
            abstand = ultraschall.distance()
            if abstand > abstandsgrenze:
                fahrwerk.drive(70,0)
                wait(300)
                abstand = ultraschall.distance()
        else:
            fahrwerk.drive(70,0)
            wait(300)
            abstand = ultraschall.distance()
    else:
        fahrwerk.drive(70,0)
        wait(300)
        abstand = ultraschall.distance()
    




    #Eine Linie nachfahren mit IR-Sensor, 
    if wert < ir_grenze:
        #ev3.speaker.beep()
        fahrwerk.stop()  # anhalten
        wait(500)
        fahrwerk.drive(-150, 0)  # vorwärts fahren
        wait(500)
        fahrwerk.drive(300, -5)      # vorwärts fahren
        wait(500)
    elif wert == ir_grenze:
        fahrwerk.drive(300, 0)
        wait (500)
    else:
        fahrwerk.stop()
        wait(500)
        fahrwerk.drive(-150, 0)
        wait(500)
        fahrwerk.drive(300, 5)
        wait(500)




#Einfaches geradeaus Fahren
ev3.speaker.beep()                        # Piepston
fahrwerk.straight(400)                   # fahre 1000 mm vorwärts
ev3.speaker.beep()                        # Piepston


#eine ausgewählte Linie folgen
ev3.speaker.beep()                        # Piepston
fahrwerk.straight(700)                    # fahre 700 mm vorwärts
fahrwerk.turn(120)
fahrwerk.straight(1000)                   # fahre 1000 mm vorwärts
fahrwerk.turn(122)
fahrwerk.straight(600)                    # fahre 700 mm vorwärts
ev3.speaker.beep()                        # Piepston

#Sensor links testen
ev3.speaker.beep()  
fahrwerk.drive(100, 0)                          # vorwärts fahren

helligkeit = licht_links.reflection()           # Helligkeit messen
while helligkeit < helligkeitsgrenze_links:     # Messwert mit Grenzwert vergleichen     
    wait(100)                                   # warte 100 ms = fahre 100 ms weiter
    helligkeit = licht_links.reflection()       # Helligkeit messen
    
fahrwerk.stop()
ev3.speaker.beep()

#Sensor rechts testen
ev3.speaker.beep()  
fahrwerk.drive(100, 0)                          # vorwärts fahren

helligkeit = licht_rechts.reflection()          # Helligkeit messen
while helligkeit < helligkeitsgrenze_links:     # Messwert mit Grenzwert vergleichen     
    wait(100)                                   # warte 100 ms = fahre 100 ms weiter
    helligkeit = licht_rechts.reflection()      # Helligkeit messen
    
fahrwerk.stop()
ev3.speaker.beep()


#Sensor Wandabstand testen
ev3.speaker.beep()  
fahrwerk.drive(100, 0)                      # vorwärts fahren

abstand = ultraschall.distance()            # Abstand messen
while abstand < abstandsgrenze:             # Messwert mit Grenzwert vergleichen     
    wait(100)                               # warte 100 ms = fahre 100 ms weiter
    abstand = ultraschall.distance()        # Abstand messen
    
fahrwerk.stop()
ev3.speaker.beep()



# antreffen einer Sackgasse 
#die Linien am boden sollen vor dem Eintreffen einer Sackgasse enden
link=licht_links.reflection()
rechts=licht_rechts.reflection()    
abstand=ultraschall.distance()


#eine ausgewählte Linie folgen (Linie in der Mitte des Roboters)
ev3.speaker.beep()  
fahrwerk.drive(100, 0)                          # vorwärts fahren
while True:
    helligkeit_links = licht_links.reflection()
    helligkeit_rechts = licht_rechts.reflection()        # Helligkeit messen
    if helligkeit_links < helligkeitsgrenze_links and helligkeit_rechts < helligkeitsgrenze_rechts:
       fahrwerk.stop()

    elif helligkeit_links > helligkeitsgrenze_links and helligkeit_rechts > helligkeitsgrenze_rechts:
       fahrwerk.drive(100, 0) 
          
    elif helligkeit_links > helligkeitsgrenze_links:
     
        fahrwerk.turn(5)
        wait(100)
    elif helligkeit_rechts > helligkeitsgrenze_rechts:
            fahrwerk.drive(100, 0) 
            fahrwerk.turn(-5)
            wait(100)
            fahrwerk.stop()
    else:
        fahrwerk.drive(100, 0)
ev3.speaker.beep()
"""

