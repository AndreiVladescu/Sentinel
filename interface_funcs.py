import cv2
import datetime
import numpy as np

def add_text_next_to_dial(img, text, dial_center, dial_size, color, font, font_scale, thickness):
    text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
    lower_right_corner = (dial_center[0] + dial_size // 2 + 5, dial_center[1] + dial_size // 2)
    text_position = (lower_right_corner[0] - 15, lower_right_corner[1] - text_size[1] // 2 - 65)
    cv2.putText(img, text, text_position, font, font_scale, color, thickness, cv2.LINE_AA)

def add_current_time(img, color, font, font_scale, thickness):
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    text_size, _ = cv2.getTextSize(current_time, font, font_scale, thickness)
    text_position = (img.shape[1] - text_size[0] - 10, text_size[1] + 10)
    cv2.putText(img, current_time, text_position, font, font_scale, color, thickness, cv2.LINE_AA)

def add_text_lower_left(img, text, color, font, font_scale, thickness):
    text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
    text_position = (10, img.shape[0] - text_size[1] - 10)
    cv2.putText(img, text, text_position, font, font_scale, color, thickness, cv2.LINE_AA)

def draw_arrow(img, angle, start_point, length, color, thickness, horizontal=True):
    if horizontal:
        end_point = (int(start_point[0] - length),
                     int(start_point[1]))
    else:
        end_point = (int(start_point[0]),
                     int(start_point[1] - length))
    cv2.line(img, start_point, end_point, color, thickness)

    # Arrowhead
    arrow_length = 10
    if horizontal:
        arrow_point = (end_point[0], end_point[1] - arrow_length)
    else:
        arrow_point = (end_point[0] - arrow_length, end_point[1])

    cv2.line(img, end_point, arrow_point, color, thickness)
