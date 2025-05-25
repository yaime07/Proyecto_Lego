#!/usr/bin/env pybricks-micropython

# Importar librerías necesarias de PyBricks
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Inicializar el EV3 Brick
ev3 = EV3Brick()

# Inicializar los motores conectados a las ruedas
Right_W = Motor(Port.D)  # Motor derecho en el puerto A
Left_W = Motor(Port.A)  # Motor izquierdo en el puerto D
Arm = Motor(Port.C) # Motor del brazo en el puerto C
Robot = DriveBase(Left_W, Right_W, wheel_diameter=68.8, axle_track=160)


# Inicializar el giroscopio en el puerto S2
GyrS = GyroSensor(Port.S2)
#Touch = TouchSensor(Port.S1)

#Funciones de movimiento fundamentales, aquí se encuentran las funciones que permiten vincular el uso de motores y sensores.
#Se tiene como nomenclatura las siguientes variables con sus fines:
#           a : Parametro para angulos
#           v : Parametro para velocidades - Potencia
#           n : Parametro de recorrido  (avance)

def avanzar(robot, distancia_mm, velocidad_mm_s):
    """
    Hace que el robot avance una distancia específica a una velocidad dada.

    Parámetros:
    robot (DriveBase): el objeto que controla los motores (por ejemplo, Robot).
    distancia_mm (int o float): la distancia a recorrer en milímetros. 
                                (positivo para adelante, negativo para atrás)
    velocidad_mm_s (int o float): la velocidad de movimiento en milímetros por segundo.

    Uso:
    avanzar(Robot, 300, 200)  # Avanza 30 cm a 20 cm/segundo
    avanzar(Robot, -200, 150) # Retrocede 20 cm a 15 cm/segundo
    """
    robot.drive_time(velocidad_mm_s, 0, abs(distancia_mm) / abs(velocidad_mm_s) * 1000)
    robot.stop()

def girar_con_gyro(robot, gyro, angulo_deseado, velocidad):
    """
    Gira el robot usando el giroscopio para ser más preciso.
    
    robot: objeto DriveBase
    gyro: objeto GyroSensor
    angulo_deseado: el ángulo que quieres girar (positivo: derecha, negativo: izquierda)
    velocidad: la velocidad del giro

    ejemplo de uso:  girar_con_gyro(Robot, GyrS, -90, 50)    # Gira 90 grados a la derecha
    """
    # Resetear gyro para empezar en 0
    gyro.reset_angle(0)
    
    # Dependiendo del signo del ángulo, girar hacia derecha o izquierda
    if angulo_deseado > 0:
        # Giro a la derecha
        while gyro.angle() < angulo_deseado:
            robot.drive(0, velocidad)  # velocidad positiva (gira)
    else:
        # Giro a la izquierda
        while gyro.angle() > angulo_deseado:
            robot.drive(0, -velocidad) # velocidad negativa (gira hacia el otro lado)

    # Frenar al terminar
    robot.stop()

def establecer_a(angulo):
    """
    Establece un nuevo ángulo de referencia para el motor Arm.

    Parámetros:
    angulo (int): El ángulo que se asignará como el nuevo 0 para el motor Arm.

    Uso:
    - Llama a establecer_a(0) para resetear el ángulo actual del brazo a 0.
    - Luego puedes mover el brazo a un ángulo relativo, por ejemplo:
        Arm.run_target(200, 90)  # mueve el brazo 90 grados desde el nuevo 0
        Arm.run_target(200, 0)   # regresa el brazo a la posición inicial

    Ejemplo de uso:
    establecer_a(0)
    Arm.run_target(200, 90)
    Arm.run_target(200, 0)
    """
    Arm.reset_angle(angulo)
    

def main():
    establecer_a(0)
    Arm.run_target(100, 40)
    #avanzar(Robot, 300, 200)

main()