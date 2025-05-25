#!/usr/bin/env pybricks-micropython

# Importar librerías necesarias de PyBricks
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep
import math

# Inicializar el EV3 Brick
ev3 = EV3Brick()

# Inicializar los motores conectados a las ruedas
Right_W = Motor(Port.D)  # Motor derecho en el puerto D
Left_W = Motor(Port.A)   # Motor izquierdo en el puerto A
Arm = Motor(Port.C)      # Motor del brazo en el puerto C
Robot = DriveBase(Left_W, Right_W, wheel_diameter=68.8, axle_track=160)

# Inicializar el giroscopio en el puerto S2
GyrS = GyroSensor(Port.S2)

# Variables globales de posición (odometría)
x = 0      # posición X en mm
y = 0      # posición Y en mm
theta = 0  # orientación θ en radianes

def actualizar_posicion(v, w, dt):
    """
    Actualiza la posición global (x, y, theta) del robot con base en su
    velocidad lineal (v) y angular (w), y un intervalo de tiempo dt.
    """
    global x, y, theta
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += w * dt

def avanzar(robot, gyro, distancia_mm, velocidad_mm_s):
    """
    Avanza o retrocede en línea recta con corrección de giroscopio.
    """
    direccion = 1 if distancia_mm > 0 else -1
    velocidad = abs(velocidad_mm_s)
    angulo_inicial = gyro.angle()
    robot.reset()

    while abs(robot.distance()) < abs(distancia_mm):
        error = gyro.angle() - angulo_inicial
        correccion = error * 1.2
        robot.drive(direccion * velocidad, -direccion * correccion)

        # Actualizar odometría
        w = -direccion * correccion * math.pi / 180  # rad/s (aproximado)
        v = direccion * velocidad
        actualizar_posicion(v, w, 0.01)

        sleep(0.01)

    robot.stop()

def girar_con_gyro(robot, gyro, angulo_deseado, velocidad):
    """
    Gira el robot usando el giroscopio para mayor precisión.
    """
    gyro.reset_angle(0)
    
    if angulo_deseado > 0:
        while gyro.angle() < angulo_deseado:
            robot.drive(0, velocidad)
    else:
        while gyro.angle() > angulo_deseado:
            robot.drive(0, -velocidad)

    robot.stop()

def establecer_a(angulo):
    """
    Establece un nuevo ángulo de referencia para el motor Arm.
    """
    Arm.reset_angle(angulo)

def mostrar_posicion():
    """
    Muestra la posición actual en pantalla del EV3.
    """
    ev3.screen.clear()
    ev3.screen.print("x: {:.1f} mm".format(x))
    ev3.screen.print("y: {:.1f} mm".format(y))
    ev3.screen.print("θ: {:.1f}°".format(math.degrees(theta)))

def main():
    avanzar(Robot, GyrS, 400, 300)
    avanzar(Robot, GyrS, -400, 300)
    mostrar_posicion()

main()
