sys_az = 90
sys_el = 0
vis_az = 82
vis_el = 90

# Whole system
left_x = 0.0
left_y = 0.0
# Just panoramic
right_x = 0.0
right_y = 0.0

fire_trg = False
laser_trg = False
fast_trg = False
slow_trg = False
ret2_home_vis = False

class ControlData:
    def __init__(self):
        self.sys_az = 90
        self.sys_el = 0
        self.vis_az = 82
        self.vis_el = 90

        # Whole system
        self.left_x = 0.0
        self.left_y = 0.0
        # Just panoramic
        self.right_x = 0.0
        self.right_y = 0.0

        self.fire_trg = False
        self.laser_trg = False
        self.fast_trg = False
        self.slow_trg = False
        self.ret2_home = False

        self.heading = -1

'''
    def __eq__(self, other):
        ok = True
        if isinstance(other, ControlData):
            ok = ok and (self.laser_trg == other.laser_trg)
            ok = ok and (self.sys_az == other.sys_az)
            ok = ok and (self.sys_el == other.sys_el)
            ok = ok and (self.vis_az == other.vis_az)
            ok = ok and (self.vis_el == other.vis_el)
            ok = ok and (self.fire_trg == other.fire_trg)'''
