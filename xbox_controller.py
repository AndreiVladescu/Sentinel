import pygame
import sys

A = 0
B = 1
X = 2
Y = 3
LEFT_BUMP = 4
RIGHT_BUMP = 5
BACK = 6
START = 7
LEFT_STICK_BTN = 8
RIGHT_STICK_BTN = 9

LEFT_STICK_X = 0
LEFT_STICK_Y = 1

RIGHT_STICK_X = 2
RIGHT_STICK_Y = 3

class Controller:
    def __init__(self, dead_zone = 0.15):
        pygame.joystick.init()
        self.joystick = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())][0]
        #self.joystick.init()
        self.dead_zone = dead_zone

    def dead_zone_adjustment(self, value):

        if value > self.dead_zone:
            return (value - self.dead_zone) / (1 - self.dead_zone)
        elif value < -self.dead_zone:
            return (value + self.dead_zone) / (1 - self.dead_zone)
        else:
            return 0

    def get_button_x(self):
        return self.joystick.get_button(0)

    def get_button_b(self):
        return self.joystick.get_button(1)

    def get_left_stick(self):

        left_stick_x = self.dead_zone_adjustment(self.joystick.get_axis(LEFT_STICK_X))
        left_stick_y = self.dead_zone_adjustment(self.joystick.get_axis(LEFT_STICK_Y))

        return left_stick_x, left_stick_y

    def get_right_stick(self):

        right_stick_x = self.dead_zone_adjustment(self.joystick.get_axis(RIGHT_STICK_X))
        right_stick_y = self.dead_zone_adjustment(self.joystick.get_axis(RIGHT_STICK_Y))

        return (right_stick_x, right_stick_y)
