#!/usr/bin/env pybricks-micropython

# Importar librerías necesarias de PyBricks
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import atan2, degrees, sqrt
from math import cos, sin, radians  # Asegúrate de importar estas funciones


# Inicializar el EV3 Brick
ev3 = EV3Brick()

# Inicializar los motores conectados a las ruedas
Right_W = Motor(Port.D)  # Motor derecho
Left_W = Motor(Port.A)   # Motor izquierdo
Arm = Motor(Port.C)      # Motor del brazo
Robot = DriveBase(Left_W, Right_W, wheel_diameter=68.8, axle_track=160)

# Inicializar el giroscopio
GyrS = GyroSensor(Port.S2)

# Variables globales de posición del robot
x = 0      # posición en mm en el eje X
y = 0      # posición en mm en el eje Y
theta = 0  # orientación en grados

# Modifica la función avanzar() para que actualice las variables globales:
def avanzar(robot, distancia_mm, velocidad_mm_s):
    global x, y, theta
    
    robot.settings(straight_speed=abs(velocidad_mm_s))
    robot.straight(distancia_mm)
    
    # Actualiza la posición basándote en el movimiento realizado
    distancia_real = distancia_mm
    radianes = radians(theta)
    x += distancia_real * cos(radianes)
    y += distancia_real * sin(radianes)



def girar_con_gyro(robot, gyro, angulo_deseado, velocidad):
    """
    Gira el robot usando el giroscopio para ser más preciso.
    """
    gyro.reset_angle(0)
    
    if angulo_deseado > 0:
        while gyro.angle() < angulo_deseado:
            robot.drive(0, velocidad)
            wait(10)
    else:
        while gyro.angle() > angulo_deseado:
            robot.drive(0, -velocidad)
            wait(10)

    robot.stop()

def establecer_a(angulo):
    """Establece un nuevo ángulo de referencia para el motor Arm."""
    Arm.reset_angle(angulo)

def ir_a(robot, gyro, x_destino,  y_destino, velocidad):
    """
    Lleva al robot a una posición (x_destino, y_destino).
    """
    global x, y, theta

    dx = x_destino - x
    dy = y_destino - y

    angulo_objetivo = degrees(atan2(dy, dx))
    delta_angulo = angulo_objetivo - theta

    # Normalizar ángulo
    while delta_angulo > 180:
        delta_angulo -= 360
    while delta_angulo < -180:
        delta_angulo += 360

    distancia = sqrt(dx*2 + dy*2)

    if abs(delta_angulo) > 90:
        # Retroceder
        girar_con_gyro(robot, gyro, delta_angulo - 180 * (1 if delta_angulo > 0 else -1), 80)
        theta = angulo_objetivo - 180
        avanzar(robot, -distancia, velocidad)
    else:
        # Avanzar
        girar_con_gyro(robot, gyro, delta_angulo, 80)
        theta = angulo_objetivo
        avanzar(robot, distancia, velocidad)

    x = x_destino
    y = y_destino

def mano():
    establecer_a(0)
    Arm.run_target(200, 160)
    wait(100)
    avanzar(Robot, -28, 85)
    Arm.run_target(200, 0)

def r1():
    global x, y, theta
    x, y, theta = 0, 0, 0    
    
    ir_a(Robot, GyrS, 470, 0, 300)
    wait(1000)
    ir_a(Robot, GyrS, 300, 0, 200)
    wait(1000)
    ir_a(Robot, GyrS, 650, -195, 300)
    wait(1000)
    
    d = 0

    ir_a(Robot, GyrS, 590, -380, 600)
    wait(1000)
    ir_a(Robot, GyrS, 760, -4, 800)
    wait(1000)



def main():
    global x, y, theta
    x, y, theta = 0, 0, 0    
    
    ir_a(Robot, GyrS, 470, 0, 300)
    wait(1000)
    ir_a(Robot, GyrS, 300, 0, 200)
    wait(1000)
    ir_a(Robot, GyrS, 650, -195, 300)
    wait(1000)
    
    d = 0

    ir_a(Robot, GyrS, 590, -380, 600)
    wait(1000)
    ir_a(Robot, GyrS, 760, -4, 800)
    wait(1000)
    
    while d < 3: 
        # En lugar de avanzar(), usa ir_a() para moverte hacia atrás
        ir_a(Robot, GyrS, x - 280*cos(radians(theta)), y - 280*sin(radians(theta)), 800)
        wait(1000)
        ir_a(Robot, GyrS, 760, -4, 800)
        wait(1000)
        d = d + 1 
    

    ir_a(Robot, GyrS, x + 100*cos(radians(theta)), y + 100*sin(radians(theta)), 800)    
    wait(1000)
    ir_a(Robot, GyrS, 900, -300, 800)


    #ir_a(Robot, GyrS, 470, 0, 300)
    #wait(1000)
    
main()