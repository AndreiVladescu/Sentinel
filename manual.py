import sys
import threading
from math import tanh
import pygame
from pygame.locals import *
import serial
from time import sleep
from xbox_controller import *
from config import *
import multiprocessing

def manual():
    ser = serial.Serial('COM8', 115200, timeout=3)
    while True:
        line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
        # Ignore the boot log messages
        if line.startswith('entry'):
            break
    print(ser.readline())
    while True:
        ser.write(bytes('A50\n', 'utf-8'))
        print(ser.readline())
        sleep(0.02)
        ser.write(bytes('A80\n', 'utf-8'))
        print(ser.readline())
        sleep(0.02)
        ser.write(bytes('a80\n', 'utf-8'))
        print(ser.readline())
        sleep(0.02)
        ser.write(bytes('a100\n', 'utf-8'))
        print(ser.readline())
        sleep(0.02)
    '''while True:
        x = input()
        #ser.write(bytes(x, 'utf-8'))
        ser.writelines(bytes(x, 'utf-8'))
        print(ser.readline())'''
def xbox_thread():
    global left_x
    global left_y
    global right_x
    global right_y
    global fire_trg
    global laser_trg
    global fast_trg
    global slow_trg
    global ret2_home

    input_sys = [0, 0]
    input_vis = [0, 0]

    pygame.init()
    pygame.joystick.init()

    while True:
        for event in pygame.event.get():
            if event.type == JOYBUTTONUP:
                #print(event.button)
                if event.button == LEFT_BUMP:
                    print("LEFT_BUMP FASTER OFF")
                    fast_trg = False
                elif event.button == RIGHT_BUMP:
                    print("RIGHT_BUMP SLOWER OFF")
                    slow_trg = False
            if event.type == JOYBUTTONDOWN:
                #print(event.button)
                if event.button == X:
                    print("X FIRE")
                    fire_trg = not fire_trg
                elif event.button == LEFT_BUMP:
                    print("LEFT_BUMP FASTER ON")
                    fast_trg = True
                elif event.button == RIGHT_BUMP:
                    print("RIGHT_BUMP  ON")
                    slow_trg = True
                elif event.button == B:
                    print("B LASER")
                    laser_trg = not laser_trg
                elif event.button == Y:
                    print("Y Ret2 Home")
                    ret2_home = True
            if event.type == JOYAXISMOTION:
                #print(event.axis)
                if event.axis < 2:
                    input_sys[event.axis] = event.value
                    left_x = input_sys[0]
                    left_y = input_sys[1]
                elif event.axis < 4:
                    input_vis[event.axis - 2] = event.value
                    right_x = input_vis[0]
                    right_y = input_vis[1]

            if event.type == JOYDEVICEADDED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
                for joystick in joysticks:
                    print(joystick.get_name())
            if event.type == JOYDEVICEREMOVED:
                joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
def xbox_aim():
    global sys_el, laser_trg, fire_trg
    global sys_az
    global vis_az
    global vis_el
    global left_x
    global left_y
    global right_x
    global right_y
    global fast_trg
    global slow_trg
    global ret2_home

    xbox_th = threading.Thread(target=xbox_thread)
    xbox_th.start()

    ser = serial.Serial('COM8', baudrate=115200)
    while True:
        line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
        # Ignore the boot log messages
        if line.startswith('entry'):
            break
    print(ser.readline())


    while True:
        input_vis = [left_x, left_y]
        input_sys = [right_x, right_y]

        modifier = 3
        #print(input_sys)
        if fast_trg:
            modifier = modifier * 2
        if slow_trg:
            modifier = modifier / 2

        az_val = input_sys[0]
        if abs(az_val) > 0.2:
            signum = az_val / abs(az_val)
            val = tanh(az_val + signum * 0.35) * modifier
            if sys_az > 15 and sys_az < 165:
                sys_az = sys_az + val
                sys_az = round(sys_az, 3)
                print('A' + str(sys_az))
                ser.write(bytes('A' + str(sys_az) + '\n', 'utf-8'))
                print(ser.readline())

        el_val = input_sys[1]
        if abs(el_val) > 0.2:
            signum = el_val / abs(el_val)
            val = tanh(el_val + signum * 0.25) * modifier
            if sys_el > -30 and sys_el < 30:
                sys_el = sys_el + val
                sys_el = round(sys_el, 3)
                print('E' + str(sys_el))
                ser.write(bytes('E' + str(sys_el) + '\n', 'utf-8'))
                print(ser.readline())

        az_val = input_vis[0]
        if abs(az_val) > 0.1:
            signum = az_val / abs(az_val)
            val = tanh(az_val + signum * 0.15) * modifier
            if vis_az > 10 and vis_az < 170:
                vis_az = vis_az + val
                vis_az = round(vis_az, 3)
                print('a' + str(vis_az))
                ser.write(bytes('a' + str(vis_az) + '\n', 'utf-8'))
                print(ser.readline())

        el_val = input_vis[1]
        if abs(el_val) > 0.1:
            signum = el_val / abs(el_val)
            val = tanh(el_val + signum * 0.15) * modifier
            if vis_el > 70 and vis_el < 110:
                vis_el = vis_el + val
                vis_el = round(vis_el, 3)
                print('e' + str(vis_el))
                ser.write(bytes('e' + str(vis_el) + '\n', 'utf-8'))
                print(ser.readline())

        if laser_trg:
            ser.write(bytes('l\n', 'utf-8'))
            print(ser.readline())
            laser_trg = False

        if fire_trg:
            ser.write(bytes('f\n', 'utf-8'))
            print(ser.readline())
            fire_trg = False

        if ret2_home:
            ret2_home = False
            ser.write(bytes('e90\n', 'utf-8'))
            ser.write(bytes('a90\n', 'utf-8'))
            print(ser.readline())
            print(ser.readline())

if __name__ == '__main__':
    # auto()
    # manual()
    xbox_aim()
