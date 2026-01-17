import numpy as np



class StndController(object):
    """
        This is the control most easy, only make to initial posicion
        and a pid for roll pitch and yaw of the robot
    """

    def __init__(self, default_stance):
        self.def_stance = default_stance

    def update_new_pos(self, state, command):
        temp = self.def_stance
        temp[2] = [command.robot_height] * 4 #add the nivel
        return temp
    
    def run(self, state, command):
        state.foot_locations = self.update_new_pos(state, command)
        return state.foot_locations

    @property
    def default_stance(self):
        return self.def_stance