#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from math import atan2, degrees, sqrt, cos, sin, radians

# Inicialización de hardware
ev3 = EV3Brick()
Right_W = Motor(Port.D)
Left_W = Motor(Port.A)
Arm = Motor(Port.C)
Robot = DriveBase(Left_W, Right_W, wheel_diameter=68.8, axle_track=160)
GyrS = GyroSensor(Port.S2)

# Variables globales de posición
x = 0
y = 0
theta = 0

def avanzar(robot, distancia_mm, velocidad_mm_s):
    global x, y, theta
    robot.settings(straight_speed=abs(velocidad_mm_s),
                   straight_acceleration=1100,
                   turn_rate=200,
                   turn_acceleration=900)
    robot.straight(distancia_mm)
    radianes = radians(theta)
    x += distancia_mm * cos(radianes)
    y += distancia_mm * sin(radianes)

def girar_con_gyro(robot, gyro, angulo_deseado, velocidad):
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

def actualizar_theta(angulo_giro):
    global theta
    theta += angulo_giro
    while theta > 180:
        theta -= 360
    while theta < -180:
        theta += 360

def establecer_a(angulo):
    Arm.reset_angle(angulo)

def ir_a(robot, gyro, x_destino, y_destino, velocidad):
    global x, y, theta
    dx = x_destino - x
    dy = y_destino - y
    angulo_objetivo = degrees(atan2(dy, dx))
    delta_angulo = angulo_objetivo - theta
    while delta_angulo > 180:
        delta_angulo -= 360
    while delta_angulo < -180:
        delta_angulo += 360
    distancia = sqrt(dx**2 + dy**2)
    if abs(delta_angulo) > 90:
        girar_con_gyro(robot, gyro, delta_angulo - 180 * (1 if delta_angulo > 0 else -1), 80)
        theta = angulo_objetivo - 180
        avanzar(robot, -distancia, velocidad)
    else:
        girar_con_gyro(robot, gyro, delta_angulo, 80)
        theta = angulo_objetivo
        avanzar(robot, distancia, velocidad)
    x = x_destino
    y = y_destino

def ruta1():
    global theta
    ir_a(Robot, GyrS, 470, 0, 300)
    wait(1000)
    ir_a(Robot, GyrS, 300, 0, 200)
    wait(1000)
    ir_a(Robot, GyrS, 700, -269, 300)
    wait(1000)
    girar_con_gyro(Robot, GyrS, -82, 80)
    actualizar_theta(-82)
    wait(1000)
    ir_a(Robot, GyrS, 780, 2, 900)
    wait(1000)
    ir_a(Robot, GyrS, 779, -50, 800)
    wait(1000)
    girar_con_gyro(Robot, GyrS, -6, 80)
    actualizar_theta(-6)
    wait(1000)
    ir_a(Robot, GyrS, 804, 190, 3500)
    wait(1000)
    ir_a(Robot, GyrS, 779, -30, 300)
    wait(1000)
    ir_a(Robot, GyrS, 799, 190, 3500)
    wait(1000)
    ir_a(Robot, GyrS, 800, -2, 300)
    wait(1000)
    ir_a(Robot, GyrS, 809, 300, 3500)
    wait(1000)
    ir_a(Robot, GyrS, 800, -2, 300)
    wait(1000)
    ir_a(Robot, GyrS, 809, 390, 3500)
    wait(1000)
    ir_a(Robot, GyrS, 800, -2, 300)
    wait(1000)
    ir_a(Robot, GyrS, 0, 0, 300)
    wait(1000)

def ruta2():
    global theta
    ir_a(Robot, GyrS, 400, -130, 300)
    wait(1000)
    girar_con_gyro(Robot, GyrS, -5, 80)
    actualizar_theta(-5)
    wait(1000)
    ir_a(Robot, GyrS, 707, -225, 300)
    wait(1000)
    ir_a(Robot, GyrS, 931, -318, 300)
    wait(1000)
    girar_con_gyro(Robot, GyrS, 7, 80)
    actualizar_theta(7)
    wait(1000)
    Arm.run_angle(200, -60)

def ruta3():
    global theta
    ir_a(Robot, GyrS, 600, 220, 890)
    wait(1000)
    girar_con_gyro(Robot, GyrS, 90, 80)
    actualizar_theta(90)
    ir_a(Robot, GyrS, 560, 290, 890)
    wait(1000)
    establecer_a(0)
    Arm.run_target(200, 75)
    wait(1000)
    ir_a(Robot, GyrS, 600, 220, 890)
    wait(1000)
    girar_con_gyro(Robot, GyrS, 132, 80)
    actualizar_theta(132)
    Arm.run_target(200, 120)
    wait(1000)
    ir_a(Robot, GyrS, 576, 150, 890)
    wait(1000)
    girar_con_gyro(Robot, GyrS, 1, 80)
    actualizar_theta(1)
    wait(1000)
    Arm.run_target(400, 60)
    wait(1000)
    girar_con_gyro(Robot, GyrS, 5, 80)
    actualizar_theta(5)
    wait(1000)
    Arm.run_target(500, 100)
    wait(1000)

def main():
    global x, y, theta
    x, y, theta = 0, 0, 0
    # Descomenta la ruta que quieras ejecutar:
    ruta1()
    ruta2()
    ruta3()

main()
