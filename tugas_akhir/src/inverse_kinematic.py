import numpy as np
from numpy.linalg.linalg import solve
import rospy

class InverseKinematic:
    def __init__(self):
        self.COM_HIP_Z = 45 / 1000
        self.COM_HIP_Y = 38.5 / 1000

        self.HIP_KNEE = 78 / 1000
        self.KNEE_ANKLE = 97 / 1000
        self.ANKLE_FOOT = 32.5 / 1000
        self.TILT = 10

        self.hip_offset = 0.0
        self.ankle_offset = 0.0

    def Rx(self, theta):
        return np.matrix([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])


    def Ry(self,theta):
        return np.matrix([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])


    def Rz(self,theta):
        return np.matrix([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]])


    def inverse_kinematic(self, COM, LEG, isLeft):
        if isLeft:
            self.COM_HIP_Y = -38.5 / 1000
        else:
            self.COM_HIP_Y = 38.5 / 1000
        p_FOOT = LEG.T
        R_FOOT = self.Rz(.0) * self.Ry(.0) * self.Rx(.0)
        p_COM = COM.T
        R_COM = self.Rz(.0) * self.Ry(self.TILT*np.pi/180) * self.Rx(.0)

        p_COM_FOOT = p_COM - p_FOOT
        
        p_ANKLE_FOOT = np.matrix([.0, .0, self.ANKLE_FOOT]).T

        p_HIP_PELVIS = np.matrix([.0, self.COM_HIP_Y, self.COM_HIP_Z]).T

        p_COM_ANKLE = p_COM_FOOT - p_ANKLE_FOOT

        r = p_COM_ANKLE - (R_COM * p_HIP_PELVIS)
        
        C = np.linalg.norm(r)

        c5 = (-self.HIP_KNEE ** 2 - self.KNEE_ANKLE ** 2 + C ** 2) / (2 * self.HIP_KNEE * self.KNEE_ANKLE)
        
        if c5 >= 1:
            q5 = 0.0
        elif c5 <= -1:
            q5 = np.pi
        else:
            q5 = np.arccos(c5)


        alpha = np.arcsin((self.HIP_KNEE / C) * np.sin(np.pi - q5))
        q7 = np.arctan2(r[1, 0], r[2, 0])
        
        if q7 > np.pi / 2:
            q7 -= np.pi
        elif q7 < -np.pi / 2:
            q7 += np.pi

        q6 = -np.arctan2(r[0, 0], np.sign(r[2, 0]) * np.sqrt(r[1, 0] ** 2 + r[2, 0] ** 2)) - alpha + self.ankle_offset

        R = R_COM.T * R_FOOT * self.Rx(-q7) * self.Ry(-q6 - q5)
        q2 = np.arctan2(-R[0, 1], R[1, 1])
        cz = np.cos(q2)
        sz = np.sin(q2)
        q3 = np.arctan2(R[2, 1], (-R[0, 1] * sz + R[1, 1] * cz))
        q4 = np.arctan2(-R[2, 0], R[2, 2]) + self.hip_offset
        
        ##----- keterangan -----
        # q3 = hip roll , q4 hip pitch , q5 = lutut , q6 = angkle pitch, q7 = angkle roll

        return np.array([q3, q4, q5, q6, q7, q2 ])#tinggal mengeluarkan q2 untk yaw

    def solve(self, COM, LEFT, RIGHT):
        # AXIS  = np.array([-1,1,-1,-1,1,1, -1,-1,1,1,1,-1]) #[-1,-1,-1,-1,1,-1,1,1,1,1] 6 kiri(7,10,12,14,16,18), 6 kanan(8,9,11,13,15,17)
        
        joint_left = self.inverse_kinematic(COM,LEFT, True)
        joint_right = self.inverse_kinematic(COM,RIGHT, False)

        # joint = np.hstack((joint_left,joint_right))

        # if joint[1] > 0:
        #     joint[1] *= 1.2
        # else :
        #     joint[1] *= 0.37

        # if joint[5] > 0:
        #     joint[5] *= 0.96
        # else :
        #     joint[5] *= 1

        # if joint[7] > 0:
        #     joint[7] *= 0.37
        # else :
        #     joint[7] *= 1.12

        # if joint[11] > 0:
        #     joint[11] *= 1
        # else :
        #     joint[11] *= 1.0016

        # return np.array(joint) #* AXIS

        
        return np.array(np.hstack((joint_left,joint_right))) 

def main():
        ik = InverseKinematic()
        FOOT_DISTANCE = 6.5 / 1000
        COM_ = np.matrix([.0, .04, 230 / 1000])
        LEFT_ = np.matrix([.0, 38.5 / 1000  + FOOT_DISTANCE, .0])
        RIGHT_ = np.matrix([.0, -38.5 / 1000 - FOOT_DISTANCE, .0])

        RES = ik.solve(COM_, LEFT_, RIGHT_)

        for i in range(0, 5):
            print("q", i , " ", (RES[i]), "\t\t\t ", (RES[i+5]))

if __name__ == "__main__":
    main()