import multiprocessing
import time
from multiprocessing import Value, Queue, Array
import sys
import threading
import cv2
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap, QStandardItem, QStandardItemModel, QFont
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QModelIndex
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from ultralytics import YOLO
import supervision as sv
# from imutils.object_detection import non_max_suppression
from ballistic_calculator import *
from pygame.locals import *
import serial
from math import tanh
from time import sleep
from xbox_controller import *
from interface_funcs import *
from multiprocessing.shared_memory import SharedMemory
from multiprocessing import Lock
import pyzed.sl as sl
from config import *
from time import sleep
import pickle

ballistic_calculator = BallisticCalculator(resolution=(1088, 640))

bw_image_switch = False
zoomed_switch = False
overlay_switch = False
targets_switch = False
camera_name = "Main Camera"
rounds_left = 30

ctrl_data = ControlData()
displayed_image = None

def xbox_thread_func():
    global left_x
    global left_y
    global right_x
    global right_y
    global fire_trg
    global fast_trg
    global slow_trg
    global ret2_home
    global laser_trg

    input_sys = [0, 0]
    input_vis = [0, 0]

    pygame.init()
    pygame.joystick.init()

    while True:
        for event in pygame.event.get():
            if event.type == JOYBUTTONUP:
                # print(event.button)
                if event.button == LEFT_BUMP:
                    print("LEFT_BUMP FASTER OFF")
                    fast_trg = False
                elif event.button == RIGHT_BUMP:
                    print("RIGHT_BUMP SLOWER OFF")
                    slow_trg = False
            if event.type == JOYBUTTONDOWN:
                # print(event.button)
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
                # print(event.axis)
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


def xbox_serial_process_func(heading_v, azimuth_v, elevation_v):
    global fire_trg
    global sys_el
    global sys_az
    global vis_az
    global vis_el
    global left_x
    global left_y
    global right_x
    global right_y
    global fast_trg
    global slow_trg
    global laser_trg

    xbox_th = threading.Thread(target=xbox_thread_func)
    xbox_th.start()

    ser = serial.Serial('COM8', baudrate=115200)

    while True:
        line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
        # Ignore the boot log messages
        print(line)
        if line.startswith('entry'):
            break
    print(ser.readline())

    ser.write(bytes('o\n', 'utf-8'))
    heading_v.value = float(ser.readline().decode().split(',')[2])
    azimuth_v.value = sys_az
    elevation_v.value = sys_el

    while True:
        input_vis = [left_x, left_y]
        input_sys = [right_x, right_y]

        modifier = 2
        # print(input_sys)
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
        if abs(az_val) > 0.2:
            val = tanh(az_val) * modifier
            if vis_az > 10 and vis_az < 170:
                vis_az = vis_az + val
                vis_az = round(vis_az, 3)
                print('a' + str(vis_az))
                ser.write(bytes('a' + str(vis_az) + '\n', 'utf-8'))
                print(ser.readline())

        el_val = input_vis[1]
        if abs(el_val) > 0.2:
            val = tanh(el_val) * modifier
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

        azimuth_v.value = sys_az
        elevation_v.value = sys_el


def apply_watermark(image, crosshair_size=350, dial_thickness=1, opacity=0.35):
    global ctrl_data

    h, w, d = image.shape[:3]
    watermark = np.zeros((h, w, d), dtype=np.uint8)
    graphic_color = (168, 98, 50)
    # Draw crosshair
    ch_length = crosshair_size // 2
    center = (w // 2, h // 2)
    cv2.line(watermark, (center[0] - ch_length, center[1]), (center[0] + ch_length, center[1]), graphic_color,
             dial_thickness)
    cv2.line(watermark, (center[0], center[1] - ch_length), (center[0], center[1] + ch_length), graphic_color,
             dial_thickness)

    # Draw square dial
    square_size = crosshair_size * 2
    top_left = (center[0] - square_size // 2, center[1] - square_size // 3)
    bottom_right = (center[0] + square_size // 2, center[1] + square_size // 3)
    cv2.rectangle(watermark, top_left, bottom_right, graphic_color, dial_thickness)

    # Draw compass headings and lines
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    line_length = 15
    # FOV = 85deg for 720p
    fov = 85
    heading = ctrl_data.heading - 90 + ctrl_data.sys_az
    start_heading = int(heading - fov / 2)
    end_heading = int(heading + fov / 2)
    segments = end_heading - start_heading
    segment_length = int(w / segments)
    angles = np.linspace(5, w - 5, num=segments)

    heading_spacing = int(w / 85)
    prev_letter = False

    for angle in angles:
        x1 = int(angle)
        y1 = 45
        x2 = x1
        offset = 15
        y2 = y1 + line_length
        temp_heading = int(start_heading + (x1 - angles[0]) * segments / (w - 5))
        watermark = cv2.line(watermark, (x1, y1), (x2, y2), graphic_color, 2)
        text = ''
        if prev_letter:
            watermark = cv2.line(watermark, (x1, y1), (x2, y2), graphic_color, 2)
            prev_letter = False
        elif temp_heading == 0 or temp_heading == 360:
            text = 'E'
            prev_letter = True
        elif temp_heading == 45:
            text = 'NE'
            prev_letter = True
        elif temp_heading == 90:
            text = 'N'
            prev_letter = True
        elif temp_heading == 135:
            text = 'NW'
            prev_letter = True
        elif temp_heading == 180:
            text = 'W'
            prev_letter = True
        elif temp_heading == 225:
            text = 'SW'
            prev_letter = True
        elif temp_heading == 270:
            text = 'S'
            prev_letter = True
        elif temp_heading == 315:
            text = 'SE'
            prev_letter = True
        if text != '':
            watermark = cv2.putText(watermark, text, (x1 - segment_length, y1 - offset), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    graphic_color, 2, cv2.LINE_AA)
        watermark = cv2.line(watermark, (x1, y1), (x2, y2), graphic_color, 2)
        # prev_letter = False

        '''if text:
            text_size, _ = cv2.getTextSize(text, font, font_scale, 2)
            text_x = x1 - text_size[0] // 2
            text_y = y2 + text_size[1] + 5
            cv2.putText(watermark, text, (text_x, text_y), font, font_scale, graphic_color, 2, cv2.LINE_AA)'''

    ctrl_data

    azimuth = int(ctrl_data.sys_az)
    elevation = int(ctrl_data.sys_el)
    azimuth_start_point = (w - 200, h - 50)
    azimuth_end_point = (w - 20, h - 50)
    elevation_start_point = (w - 50, h - 130)
    elevation_end_point = (w - 50, h - 70)

    az_center_coordinates = (w - 200 + azimuth, h - 50)
    el_center_coordinates = (w - 50, h - 100 + elevation)

    watermark = cv2.circle(watermark, az_center_coordinates, 5, graphic_color, 2)
    watermark = cv2.circle(watermark, el_center_coordinates, 5, graphic_color, 2)

    watermark = cv2.line(watermark, azimuth_start_point, azimuth_end_point, graphic_color, 5)
    watermark = cv2.line(watermark, elevation_start_point, elevation_end_point, graphic_color, 5)

    # text_to_add = str(rounds_left) + " Rounds Left"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    font_thickness = 1
    text_color = graphic_color

    # add_text_next_to_dial(watermark, text_to_add, center, crosshair_size * 2, text_color, font, font_scale,
    #                      font_thickness)

    text_to_add = "Distance " + str(round(distance, 1)) + "m"
    add_text_next_to_dial(watermark, text_to_add, (center[0], center[1] + 30), crosshair_size * 2, text_color, font,
                          font_scale,
                          font_thickness)

    add_current_time(watermark, text_color, font, font_scale, font_thickness)

    text_to_add_lower_left = camera_name
    add_text_lower_left(watermark, text_to_add_lower_left, text_color, font, font_scale, font_thickness)

    # Apply watermark to the image
    result = cv2.addWeighted(image, 1 - opacity, watermark, opacity, 0)
    return result


class CameraCaptureThread(QThread):
    frame_signal = pyqtSignal(QImage)

    def __init__(self, camera_id=0):
        super().__init__()
        self.camera_id = camera_id
        self.is_running = True

    def run(self):
        global heading
        global ctrl_data
        global azimuth
        global elevation
        global distance
        global displayed_image
        global centers
        global bboxes

        zed = sl.Camera()

        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD1080
        init.depth_mode = sl.DEPTH_MODE.ULTRA
        init.coordinate_units = sl.UNIT.METER
        err = zed.open(init)

        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            zed.close()
            exit(1)
        runtime = sl.RuntimeParameters()
        runtime.sensing_mode = sl.SENSING_MODE.STANDARD

        # Prepare new image size to retrieve half-resolution images
        image_size = zed.get_camera_information().camera_resolution
        img_width = 1591
        img_height = 881
        image_size.width = img_width
        image_size.height = img_height
        # ballistic_calculator = BallisticCalculator(resolution=(int(image_size.width), int(image_size.height)))

        half_width = int(img_width / 2)
        half_height = int(img_height / 2)

        print(half_width, half_height)
        # Declare your sl.Mat matrices
        image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
        depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
        depth_map = sl.Mat(image_size.width, image_size.height)
        distance = 4000

        # cap = cv2.VideoCapture(self.camera_id)
        while heading.value == 0:
            sleep(0.1)

        ctrl_data.heading = heading.value

        while self.is_running:
            try:
                err = zed.grab(runtime)
                if err == sl.ERROR_CODE.SUCCESS:
                    zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
                    zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
                    zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

                    distance = depth_map.get_value(half_width, half_height)
                    distance = distance[1]

                    if math.isnan(distance) or math.isinf(distance):
                        distance = -1

                    if bw_image_switch:
                        depth_image_cv = depth_image_zed.get_data()
                        depth_image_cv = cv2.cvtColor(depth_image_cv, cv2.COLOR_BGR2GRAY)
                    else:
                        left_image = image_zed.get_data()
                        left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)

                        ctrl_data.sys_az = azimuth.value
                        ctrl_data.sys_el = elevation.value

                        left_image = apply_watermark(left_image)
                        if overlay_switch:
                            for center in centers:
                                left_image = cv2.circle(left_image, center, 2, (0, 0, 255), 2)

                    cv2.waitKey(1)
                    channel_width = 1
                    if not bw_image_switch:
                        channel_width = left_image.shape[2]

                    bytes_per_line = channel_width * img_width

                    if bw_image_switch:
                        qt_image = QImage(depth_image_cv.data, img_width, img_height, bytes_per_line,
                                          QImage.Format_Grayscale8)
                    else:
                        displayed_image = left_image
                        qt_image = QImage(left_image.data, img_width, img_height, bytes_per_line,
                                          QImage.Format_RGB888)

                    self.frame_signal.emit(qt_image)
            except:
                continue

    def stop(self):
        self.is_running = False


class CameraApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        # self.showMaximized()

    def init_ui(self):
        # Create QLabel to display the camera feed
        pass


class Ui_MainWindow(object):
    def btn_overlay_func(self):
        global overlay_switch
        overlay_switch = not overlay_switch

    def btn_home1_func(self):
        pass

    def btn_home2_func(self):
        pass

    def btn_up2_func(self):
        pass

    def btn_down2_func(self):
        pass

    def btn_left2_func(self):
        pass

    def btn_right2_func(self):
        pass

    def btn_up1_func(self):
        pass

    def btn_down1_func(self):
        pass

    def btn_left1_func(self):
        pass

    def btn_right1_func(self):
        pass

    def btn_forget_func(self):
        pass

    def btn_main_func(self):
        global camera_name
        camera_name = "Main Camera"

    def btn_a_func(self):
        global camera_name
        camera_name = "'A' Camera"

    def btn_b_func(self):
        global camera_name
        camera_name = "'B' Camera"

    def btn_toggle_func(self):
        global bw_image_switch
        bw_image_switch = not bw_image_switch

    def btn_supervized_func(self):
        pass

    def btn_engage_func(self):
        pass

    def btn_manual_func(self):
        pass

    def btn_auto_func(self):
        pass

    def btn_laser_func(self):
        pass

    def update_frame(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)
        self.imgLabel.setPixmap(pixmap)

    def closeEvent(self, event):
        self.camera_thread.stop()
        self.camera_thread.wait()
        event.accept()

    def on_clicked(self, index):
        item = self.model.itemFromIndex(index)
        print(item.text())

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1920, 1000)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.imageFrame = QtWidgets.QFrame(self.centralwidget)
        self.imageFrame.setGeometry(QtCore.QRect(0, 0, 1591, 881))
        self.imageFrame.setAutoFillBackground(True)
        self.imageFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.imageFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.imageFrame.setObjectName("imageFrame")

        self.imgLabel = QLabel(self.imageFrame)
        self.imgLabel.setGeometry(0, 0, 1591, 881)

        self.camera_thread = CameraCaptureThread()
        self.camera_thread.frame_signal.connect(self.update_frame)
        self.camera_thread.start()

        self.rightFrame = QtWidgets.QFrame(self.centralwidget)
        self.rightFrame.setGeometry(QtCore.QRect(1590, 0, 331, 1061))
        self.rightFrame.setAutoFillBackground(False)
        self.rightFrame.setStyleSheet("background-color: rgb(184, 184, 184);")
        self.rightFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.rightFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.rightFrame.setObjectName("rightFrame")
        self.engageBtn = QtWidgets.QPushButton(self.rightFrame)
        self.engageBtn.setGeometry(QtCore.QRect(20, 870, 281, 121))
        font = QtGui.QFont()
        font.setPointSize(32)
        self.engageBtn.setFont(font)
        self.engageBtn.setAcceptDrops(False)
        self.engageBtn.setStyleSheet("background-color: rgb(195, 15, 15);\n"
                                     "border-radius: 5px")
        self.engageBtn.setObjectName("engageBtn")
        self.mainBtn = QtWidgets.QPushButton(self.rightFrame)
        self.mainBtn.setGeometry(QtCore.QRect(30, 660, 131, 91))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(28)
        self.mainBtn.setFont(font)
        self.mainBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                   "border-radius: 5px")
        self.mainBtn.setObjectName("mainBtn")
        self.toggleBtn = QtWidgets.QPushButton(self.rightFrame)
        self.toggleBtn.setGeometry(QtCore.QRect(170, 660, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.toggleBtn.setFont(font)
        self.toggleBtn.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.toggleBtn.setAcceptDrops(False)
        self.toggleBtn.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.toggleBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                     "border-radius: 5px;\n"
                                     "")
        self.toggleBtn.setObjectName("toggleBtn")
        self.aBtn = QtWidgets.QPushButton(self.rightFrame)
        self.aBtn.setGeometry(QtCore.QRect(30, 760, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(28)
        self.aBtn.setFont(font)
        self.aBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                "border-radius: 5px")
        self.aBtn.setObjectName("aBtn")
        self.bBtn = QtWidgets.QPushButton(self.rightFrame)
        self.bBtn.setGeometry(QtCore.QRect(170, 760, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(28)
        self.bBtn.setFont(font)
        self.bBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                "border-radius: 5px")
        self.bBtn.setObjectName("bBtn")
        self.supBtn = QtWidgets.QPushButton(self.rightFrame)
        self.supBtn.setGeometry(QtCore.QRect(30, 120, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.supBtn.setFont(font)
        self.supBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                  "border-radius: 5px")
        self.supBtn.setObjectName("supBtn")
        self.laserBtn = QtWidgets.QPushButton(self.rightFrame)
        self.laserBtn.setGeometry(QtCore.QRect(190, 120, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.laserBtn.setFont(font)
        self.laserBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.laserBtn.setObjectName("laserBtn")
        self.autoBtn = QtWidgets.QPushButton(self.rightFrame)
        self.autoBtn.setGeometry(QtCore.QRect(190, 10, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.autoBtn.setFont(font)
        self.autoBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                   "border-radius: 5px")
        self.autoBtn.setObjectName("autoBtn")
        self.manualBtn = QtWidgets.QPushButton(self.rightFrame)
        self.manualBtn.setGeometry(QtCore.QRect(30, 10, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.manualBtn.setFont(font)
        self.manualBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                     "border-radius: 5px")
        self.manualBtn.setObjectName("manualBtn")
        self.targetListView = QtWidgets.QListView(self.rightFrame)
        self.targetListView.setGeometry(QtCore.QRect(30, 290, 281, 361))
        self.model = QStandardItemModel()
        self.targetListView.setModel(self.model)
        self.targetFont = QFont()
        self.targetFont.setPointSize(20)
        self.targetListView.clicked[QModelIndex].connect(self.on_clicked)
        values = ['one', 'two', 'three', 'one', 'two', 'three', 'one', 'two', 'three', 'one', 'two', 'three']

        for i in values:
            target = QStandardItem(i)
            target.setEditable(False)
            target.setFont(self.targetFont)
            self.model.appendRow(target)

        self.targetListView.setObjectName("targetListView")
        self.label_3 = QtWidgets.QLabel(self.rightFrame)
        self.label_3.setGeometry(QtCore.QRect(90, 250, 181, 31))

        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.botFrame = QtWidgets.QFrame(self.centralwidget)
        self.botFrame.setGeometry(QtCore.QRect(0, 880, 1591, 161))
        self.botFrame.setAutoFillBackground(False)
        self.botFrame.setStyleSheet("background-color: rgb(185, 185, 185);")
        self.botFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.botFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.botFrame.setObjectName("botFrame")
        self.home2Btn = QtWidgets.QPushButton(self.botFrame)
        self.home2Btn.setGeometry(QtCore.QRect(40, 20, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.home2Btn.setFont(font)
        self.home2Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.home2Btn.setObjectName("home2Btn")
        self.forgetBtn = QtWidgets.QPushButton(self.botFrame)
        self.forgetBtn.setGeometry(QtCore.QRect(640, 20, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.forgetBtn.setFont(font)
        self.forgetBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                     "border-radius: 5px")
        self.forgetBtn.setObjectName("forgetBtn")
        self.home1Btn = QtWidgets.QPushButton(self.botFrame)
        self.home1Btn.setGeometry(QtCore.QRect(990, 20, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.home1Btn.setFont(font)
        self.home1Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.home1Btn.setObjectName("home1Btn")
        self.overlayBtn = QtWidgets.QPushButton(self.botFrame)
        self.overlayBtn.setGeometry(QtCore.QRect(1440, 20, 131, 91))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.overlayBtn.setFont(font)
        self.overlayBtn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                      "border-radius: 5px")
        self.overlayBtn.setObjectName("overlayBtn")
        self.up2Btn = QtWidgets.QPushButton(self.botFrame)
        self.up2Btn.setGeometry(QtCore.QRect(440, 10, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.up2Btn.setFont(font)
        self.up2Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                  "border-radius: 5px")
        self.up2Btn.setObjectName("up2Btn")
        self.right2Btn = QtWidgets.QPushButton(self.botFrame)
        self.right2Btn.setGeometry(QtCore.QRect(510, 40, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.right2Btn.setFont(font)
        self.right2Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                     "border-radius: 5px")
        self.right2Btn.setObjectName("right2Btn")
        self.left2Btn = QtWidgets.QPushButton(self.botFrame)
        self.left2Btn.setGeometry(QtCore.QRect(370, 40, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.left2Btn.setFont(font)
        self.left2Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.left2Btn.setObjectName("left2Btn")
        self.down2Btn = QtWidgets.QPushButton(self.botFrame)
        self.down2Btn.setGeometry(QtCore.QRect(440, 70, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.down2Btn.setFont(font)
        self.down2Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.down2Btn.setObjectName("down2Btn")
        self.down1Btn = QtWidgets.QPushButton(self.botFrame)
        self.down1Btn.setGeometry(QtCore.QRect(1220, 70, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.down1Btn.setFont(font)
        self.down1Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.down1Btn.setObjectName("down1Btn")
        self.right1Btn = QtWidgets.QPushButton(self.botFrame)
        self.right1Btn.setGeometry(QtCore.QRect(1290, 30, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.right1Btn.setFont(font)
        self.right1Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                     "border-radius: 5px")
        self.right1Btn.setObjectName("right1Btn")
        self.left1Btn = QtWidgets.QPushButton(self.botFrame)
        self.left1Btn.setGeometry(QtCore.QRect(1150, 30, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.left1Btn.setFont(font)
        self.left1Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                    "border-radius: 5px")
        self.left1Btn.setObjectName("left1Btn")
        self.up1Btn = QtWidgets.QPushButton(self.botFrame)
        self.up1Btn.setGeometry(QtCore.QRect(1220, 10, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.up1Btn.setFont(font)
        self.up1Btn.setStyleSheet("background-color: rgb(230, 230, 230);\n"
                                  "border-radius: 5px")
        self.up1Btn.setObjectName("up1Btn")
        self.label = QtWidgets.QLabel(self.botFrame)
        self.label.setGeometry(QtCore.QRect(210, 10, 171, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.botFrame)
        self.label_2.setGeometry(QtCore.QRect(830, 10, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        MainWindow.setCentralWidget(self.centralwidget)

        # Functions
        self.toggleBtn.clicked.connect(self.btn_toggle_func)
        self.mainBtn.clicked.connect(self.btn_main_func)
        self.autoBtn.clicked.connect(self.btn_auto_func)
        self.aBtn.clicked.connect(self.btn_a_func)
        self.bBtn.clicked.connect(self.btn_b_func)
        self.laserBtn.clicked.connect(self.btn_laser_func)
        self.manualBtn.clicked.connect(self.btn_manual_func)
        self.supBtn.clicked.connect(self.btn_supervized_func)
        self.engageBtn.clicked.connect(self.btn_engage_func)
        self.forgetBtn.clicked.connect(self.btn_forget_func)
        self.up1Btn.clicked.connect(self.btn_up1_func)
        self.down1Btn.clicked.connect(self.btn_down1_func)
        self.left1Btn.clicked.connect(self.btn_left1_func)
        self.right1Btn.clicked.connect(self.btn_right1_func)
        self.up2Btn.clicked.connect(self.btn_up2_func)
        self.down2Btn.clicked.connect(self.btn_down2_func)
        self.left2Btn.clicked.connect(self.btn_left2_func)
        self.right2Btn.clicked.connect(self.btn_right2_func)
        self.overlayBtn.clicked.connect(self.btn_overlay_func)
        self.home1Btn.clicked.connect(self.btn_home1_func)
        self.home2Btn.clicked.connect(self.btn_home2_func)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Sentinel"))
        self.engageBtn.setText(_translate("MainWindow", "Engage"))
        self.mainBtn.setText(_translate("MainWindow", "MAIN"))
        self.toggleBtn.setText(_translate("MainWindow", "TOGGLE"))
        self.aBtn.setText(_translate("MainWindow", "A"))
        self.bBtn.setText(_translate("MainWindow", "B"))
        self.supBtn.setText(_translate("MainWindow", "SUPERVIZED"))
        self.laserBtn.setText(_translate("MainWindow", "LASER"))
        self.autoBtn.setText(_translate("MainWindow", "AUTO"))
        self.manualBtn.setText(_translate("MainWindow", "MANUAL"))
        self.label_3.setText(_translate("MainWindow", "Detected Targets"))
        self.home2Btn.setText(_translate("MainWindow", "HOME\n"
                                                       "POSITION"))
        self.forgetBtn.setText(_translate("MainWindow", "FORGET \n"
                                                        "TARGET"))
        self.home1Btn.setText(_translate("MainWindow", "HOME\n"
                                                       "POSITION"))
        self.overlayBtn.setText(_translate("MainWindow", "SHOW \n"
                                                         "DETECTION \n"
                                                         "OVERLAY"))
        self.up2Btn.setText(_translate("MainWindow", "UP"))
        self.right2Btn.setText(_translate("MainWindow", "RIGHT"))
        self.left2Btn.setText(_translate("MainWindow", "LEFT"))
        self.down2Btn.setText(_translate("MainWindow", "DOWN"))
        self.down1Btn.setText(_translate("MainWindow", "DOWN"))
        self.right1Btn.setText(_translate("MainWindow", "RIGHT"))
        self.left1Btn.setText(_translate("MainWindow", "LEFT"))
        self.up1Btn.setText(_translate("MainWindow", "UP"))
        self.label.setText(_translate("MainWindow", "Platform Control"))
        self.label_2.setText(_translate("MainWindow", "Visor Control"))


def ml_thread_func():
    global centers
    global bboxes

    model = YOLO('yolov8m.pt')  # load an official detection model
    while True:
        if overlay_switch:
            results = model.track(source=displayed_image, tracker="tracker.yaml", device=0)
            result = results[0]
            detections = sv.Detections.from_yolov8(result)
            # detections = detections[detections.class_id == 0]
            centers = []
            bboxes = detections.xyxy
            for box in detections.xyxy:
                print(box)
                centers.append((int(box[0] + (box[2] - box[0]) / 2), int(box[1] + (box[3] - box[1]) / 2)))
            box_annotator = sv.BoxAnnotator()
            det_nmbr = len(detections.xyxy)

            labels = [
                f"{i}"
                for i
                in range(0, det_nmbr)
            ]

            annotated_image = box_annotator.annotate(displayed_image.copy(), detections=detections, labels=labels)
            cv2.circle(annotated_image, (int(annotated_image.shape[1] / 2), int(annotated_image.shape[0] / 2)), 2,
                       (0, 255, 0), 4)
            print(centers)
            '''
            for center in centers:
                cv2.circle(annotated_image, center, 2, (0, 0, 255), 2)
                angles = ballistic_computer.get_camera_angles(center[0], center[1])
                print(angles)'''

            cv2.waitKey(1)

if __name__ == '__main__':
    global heading
    global azimuth
    global elevation

    heading = Value('f', 0.0)
    azimuth = Value('f', 0.0)
    elevation = Value('f', 0.0)

    laser_lock = Lock()
    tigger_lock = Lock()

    xbox_serial_process = multiprocessing.Process(target=xbox_serial_process_func, args=(heading, azimuth, elevation))
    xbox_serial_process.start()

    ml_thread = threading.Thread(target=ml_thread_func)
    ml_thread.start()

    app = QApplication(sys.argv)
    main_window = CameraApp()
    ui = Ui_MainWindow()
    ui.setupUi(main_window)
    main_window.show()
    # xbox_process.join()
    # serial_process.join()
    sys.exit(app.exec_())
