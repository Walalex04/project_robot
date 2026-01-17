
import numpy as np 
from .RobotState import RobotState, Command, BehaviorState

from .StndController import StndController

class Robot(object): 
    """
        Principal control of the robot
    """

    def __init__(self, body, legs):
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.default_height = 0.6
        self.x_shift_front = -0.1 
        self.x_shift_back = -0.1     

        self.standController = StndController(self.default_stance)
       

        self.currentController = self.standController
        self.state = RobotState(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)


    def run(self):
        return self.currentController.run(self.state, self.command)



    @property
    def default_stance(self):
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])