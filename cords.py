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

# Variables globales de posición del robot
x = 0      # posición en mm en el eje X
y = 0      # posición en mm en el eje Y
theta = 0  # orientación en radianes


# Funciones de movimiento fundamentales

def avanzar(robot, gyro, distancia_mm, velocidad_mm_s):
    """
    Avanza o retrocede en línea recta con corrección de giroscopio.

    Parámetros:
    robot (DriveBase): Objeto que controla los motores.
    gyro (GyroSensor): Sensor de giro.
    distancia_mm (float): Distancia a recorrer (positiva o negativa).
    velocidad_mm_s (float): Velocidad absoluta en mm/s.

    Uso:
    avanzar(Robot, GyrS, 300, 150)   # Avanza 30 cm
    avanzar(Robot, GyrS, -200, 100)  # Retrocede 20 cm
    """
    global x, y, theta
    direccion = 1 if distancia_mm > 0 else -1
    velocidad = abs(velocidad_mm_s)
    angulo_inicial = gyro.angle()
    robot.reset()

    while abs(robot.distance()) < abs(distancia_mm):
        error = gyro.angle() - angulo_inicial
        correccion = error * 1.2
        robot.drive(direccion * velocidad, -direccion * correccion)

        # Actualizar odometría
        v = direccion * velocidad
        w = -direccion * correccion * math.pi / 180  # aprox. en rad/s
        dt = 0.01
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt

        sleep(dt)

    robot.stop()

def girar_con_gyro(robot, gyro, angulo_deseado, velocidad):
    """
    Gira el robot usando el giroscopio para ser más preciso.
    
    robot: objeto DriveBase
    gyro: objeto GyroSensor
    angulo_deseado: el ángulo que quieres girar (positivo: derecha, negativo: izquierda)
    velocidad: la velocidad del giro

    ejemplo de uso:  
    girar_con_gyro(Robot, GyrS, -90, 50)    # Gira 90 grados a la derecha
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

    Parámetros:
    angulo (int): El ángulo que se asignará como el nuevo 0 para el motor Arm.

    Uso:
    establecer_a(0)
    Arm.run_target(200, 90)
    Arm.run_target(200, 0)
    """
    Arm.reset_angle(angulo)

def ir_a(robot, gyro, x_destino, y_destino, velocidad):
    """
    Mueve el robot desde su posición actual (x, y) hacia las coordenadas (x_destino, y_destino).

    Parámetros:
    robot (DriveBase): El objeto del robot.
    gyro (GyroSensor): Sensor de orientación.
    x_destino (float): Coordenada X a alcanzar en mm.
    y_destino (float): Coordenada Y a alcanzar en mm.
    velocidad (float): Velocidad de movimiento en mm/s.

    Uso:
    ir_a(Robot, GyrS, 300, 300, 150)
    """
    global x, y, theta

    dx = x_destino - x
    dy = y_destino - y

    angulo_objetivo = math.atan2(dy, dx)
    angulo_giro = math.degrees(angulo_objetivo - theta)

    # Normalizar el ángulo entre -180 y 180
    angulo_giro = (angulo_giro + 180) % 360 - 180

    # Girar hacia la dirección deseada
    girar_con_gyro(robot, gyro, angulo_giro, 50)

    # Avanzar la distancia requerida
    distancia = math.hypot(dx, dy)
    avanzar(robot, gyro, distancia, velocidad)

def mostrar_posicion():
    """
    Muestra la posición actual en pantalla del EV3.
    """
    ev3.screen.clear()
    ev3.screen.print("x: {:.1f} mm".format(x))
    ev3.screen.print("y: {:.1f} mm".format(y))
    ev3.screen.print("θ: {:.1f}°".format(math.degrees(theta)))


# Función principal para probar el nuevo movimiento
def main():
    ir_a(Robot, GyrS, 300, 0, 150)
    sleep(1000)
    ir_a(Robot, GyrS, 300, 300, 150)
    sleep(1000)
    ir_a(Robot, GyrS, 0, 300, 150)
    sleep(1000)
    ir_a(Robot, GyrS, 0, 0, 150)
    mostrar_posicion()

main()
