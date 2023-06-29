import math

class BallisticCalculator():
    """
        Ballistic Calculator for orienting an actuator in the direction of
    the camera's optical axis
    """
    def __init__(self, focal_length=1400, resolution=(1920, 1080),
                 system_vertical_offset=0.04, system_horizontal_offset=0.02):
        self.resolution = resolution
        self.half_res_h = resolution[0] / 2
        self.half_res_v = resolution[1] / 2
        self.system_vertical_offset = system_vertical_offset
        self.system_horizontal_offset = system_horizontal_offset
        self.focal_length = focal_length # in pixels

        self.fov_h = 2 * math.atan(resolution[0] / (2 * focal_length)) * (180 / math.pi)
        self.fov_v = 2 * math.atan(resolution[1] / (2 * focal_length)) * (180 / math.pi)

    def get_horizontal_angle(self, y_coordinate):
        angle_h = (y_coordinate - self.half_res_h) / self.half_res_h
        angle_h = angle_h * (self.fov_h / 2)
        #angle_h = angle_h
        return angle_h

    def get_vertical_angle(self, x_coordinate):
        angle_v = (x_coordinate - self.half_res_v) / self.half_res_v
        angle_v = angle_v * (self.fov_v / 2)
        #angle_v = angle_v
        return angle_v

    def get_camera_angles(self, y_coordinates, x_coordinates):
        """
           Returns angles from which the camera would need to be oriented to get the object
       at it's optical axis
       """
        return self.get_horizontal_angle(y_coordinates), self.get_vertical_angle(x_coordinates)

    def get_system_horizontal_angles(self, distance_to_target):
        # No horizontal bullet drop
        angle_h = 90 - math.degrees(math.atan(distance_to_target / self.system_horizontal_offset))
        return angle_h

    def get_system_vertical_angles(self, distance_to_target):
        # Assume no bullet drop
        angle_v = 90 - math.degrees(math.atan(distance_to_target / self.system_vertical_offset))
        return angle_v

    def get_system_angles(self, distance_to_target):
        return self.get_system_vertical_angles(distance_to_target), self.get_system_horizontal_angles(distance_to_target)