

import numpy as np 
from math import sqrt, cos, sin, acos, pi, atan2
from Transformations import homog_transform_inverse, homog_transform 


class InverseKinematics(object):
    """
        Define the behaviour of the manipulator
    """
    def __init__(self, body_dim, leg_dim):

        """
            initilize the manipulator

        """

        #body dimension
        self.body_legth = body_dim[0] 
        self.body_width = body_dim[1]

        #leg dimension
        self.l1 = leg_dim[0]
        self.l2 = leg_dim[1]  # l0
        self.l3 = leg_dim[2]  # l1
        self.l4 = leg_dim[3]  # l2
    
    def get_local_position(self, leg_pos, dx,dy,dz, roll, pitch, yaw):
        """
            get the position respect to leg's reference system (q0)
        """

        leg_pos = (np.block([[leg_pos], [np.array([1,1,1,1])]])).T

        #trasnformation matrix from base world to robot's system reference
        T_refw2crob = homog_transform(dx, dy, dz, roll, pitch, yaw)
        

        T_crob2FR = homog_transform(0.5 * self.body_legth, -0.5* self.body_width,
                    0, pi/2, 0,-pi/2)
        T_refw2FR = np.dot(T_refw2crob, T_crob2FR) 
        

        T_crob2FL = homog_transform(0.5 * self.body_legth, 0.5* self.body_width,
                    0, pi/2, 0,-pi/2)
        T_refw2FL = np.dot(T_refw2crob, T_crob2FL)

        T_crob2RR = homog_transform(-0.5 * self.body_legth, -0.5* self.body_width,
                    0, pi/2, 0,-pi/2)
        T_refw2RR = np.dot(T_refw2crob, T_crob2RR)  

        T_crob2RL = homog_transform(-0.5 * self.body_legth, 0.5* self.body_width,
                    0, pi/2, 0,-pi/2)
        T_refw2RL = np.dot(T_refw2crob, T_crob2RL) 
 

        pos_FR = np.dot(homog_transform_inverse(T_refw2FR), leg_pos[0])
        pos_FL = np.dot(homog_transform_inverse(T_refw2FL), leg_pos[1])
        pos_RR = np.dot(homog_transform_inverse(T_refw2RR), leg_pos[2])
        pos_RL = np.dot(homog_transform_inverse(T_refw2RL), leg_pos[3])

        return(np.array([pos_FR[:3],pos_FL[:3],pos_RR[:3],pos_RL[:3]]))



    def inverse_kinematic(self, leg_pos, dx,dy,dz, roll, pitch, yaw):

        positions = self.get_local_position(leg_pos,dx,dy,dz,roll,pitch,yaw)
    

        angles = [] # obtaing the angles of the

        for i in range(4): #for the 4 legs
          
            x = positions[i][0]
            y = positions[i][1]
            z = positions[i][2]
            print((x,y,z))
            h = sqrt((x ** 2 + y ** 2) - self.l2 ** 2) 
            theta = atan2(h, self.l2)

            gamma = atan2(y, x)
            q0 = theta - gamma

            d = sqrt(z ** 2 + y ** 2)
            cosq2 = (d**2 - self.l3 **2 - self.l4 **2) / (  2 * self.l3 * self.l4)
            
            sinq2 = sqrt(1 - (cosq2 ** 2))

            q2 = atan2(sinq2, cosq2)
            

            betha = atan2(y, -z)
            alpha = atan2(self.l3 * sinq2, self.l2 + self.l3 + cosq2)

            q1 = betha - alpha

        
            angles.append(q0)
            angles.append(q1)
            angles.append(q2)

        return angles



            