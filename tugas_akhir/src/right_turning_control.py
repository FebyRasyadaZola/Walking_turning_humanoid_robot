#! /usr/bin/python3

import rospy
import pandas as pd
from openpyxl import Workbook
from geometry_msgs.msg import Vector3

from inverse_kinematic import *
from servo_controller import *

def bezier_curve_2D(phase, p1, p2):
    
    x = (1-phase) * p1[0,0] + phase * p2[0,0]
    y = (1-phase) * p1[0,1] + phase * p2[0,1]
    z = (1-phase) * p1[0,2] + phase * p2[0,2]

    return np.matrix([x,y,z])

def bezier_curve_3D(phase, p1, p2, p3):
    
    x = ((1 - phase)**2 * p1[0,0]) + (2 * (1 - phase) * phase * p2[0,0]) + (phase**2 * p3[0,0])
    y = ((1 - phase)**2 * p1[0,1]) + (2 * (1 - phase) * phase * p2[0,1]) + (phase**2 * p3[0,1])
    z = ((1 - phase)**2 * p1[0,2]) + (2 * (1 - phase) * phase * p2[0,2]) + (phase**2 * p3[0,2])

    return np.matrix([x,y,z])

def bezier_curve_4D(phase, p1, p2, p3, p4):
    
    x = ((1 - phase)**3 * p1[0,0]) + (3 * (1 - phase)**2 * phase * p2[0,0]) + (3 * (1-phase) * (phase**2) * p3[0,0]) + (phase**3 * p4[0,0])
    y = ((1 - phase)**3 * p1[0,1]) + (3 * (1 - phase)**2 * phase * p2[0,1]) + (3 * (1-phase) * (phase**2) * p3[0,1]) + (phase**3 * p4[0,1])
    z = ((1 - phase)**3 * p1[0,2]) + (3 * (1 - phase)**2 * phase * p2[0,2]) + (3 * (1-phase) * (phase**2) * p3[0,2]) + (phase**3 * p4[0,2])

    return np.matrix([x,y,z])

def bezier_curve_6D(phase, p1, p2, p3, p4, p5, p6):

    x = ((1 - phase)**5 * p1[0,0]) + (5 * (1 - phase)**4 * phase * p2[0,0]) + (10 * (1 - phase)**3 * (phase**2) * p3[0,0]) + (10 * (1 - phase)**2 * (phase**3) * p4[0,0]) + (5 * (1 - phase) * (phase**4) * p5[0,0]) + ((phase**5) * p6[0,0])
    y = ((1 - phase)**5 * p1[0,1]) + (5 * (1 - phase)**4 * phase * p2[0,1]) + (10 * (1 - phase)**3 * (phase**2) * p3[0,1]) + (10 * (1 - phase)**2 * (phase**3) * p4[0,1]) + (5 * (1 - phase) * (phase**4) * p5[0,1]) + ((phase**5) * p6[0,1])
    z = ((1 - phase)**5 * p1[0,2]) + (5 * (1 - phase)**4 * phase * p2[0,2]) + (10 * (1 - phase)**3 * (phase**2) * p3[0,2]) + (10 * (1 - phase)**2 * (phase**3) * p4[0,2]) + (5 * (1 - phase) * (phase**4) * p5[0,2]) + ((phase**5) * p6[0,2])

    return np.matrix([x,y,z])

def bezier_curve_8D(phase, p1, p2, p3, p4, p5, p6, p7, p8):

    x = ((1 - phase)**7 * p1[0,0]) + (7 * (1 - phase)**6 * phase * p2[0,0]) + (21 * (1 - phase)**5 * (phase**2) * p3[0,0]) + (35 * (1 - phase)**4 * (phase**3) * p4[0,0]) + (35 * (1 - phase)**3 * (phase**4) * p5[0,0]) + (21 * (1 - phase)**2 * (phase**5) * p6[0,0]) + (7 * (1 - phase) * (phase**6) * p7[0,0]) + ((phase**7) * p8[0,0])
    y = ((1 - phase)**7 * p1[0,1]) + (7 * (1 - phase)**6 * phase * p2[0,1]) + (21 * (1 - phase)**5 * (phase**2) * p3[0,1]) + (35 * (1 - phase)**4 * (phase**3) * p4[0,1]) + (35 * (1 - phase)**3 * (phase**4) * p5[0,1]) + (21 * (1 - phase)**2 * (phase**5) * p6[0,1]) + (7 * (1 - phase) * (phase**6) * p7[0,1]) + ((phase**7) * p8[0,1])
    z = ((1 - phase)**7 * p1[0,2]) + (7 * (1 - phase)**6 * phase * p2[0,2]) + (21 * (1 - phase)**5 * (phase**2) * p3[0,2]) + (35 * (1 - phase)**4 * (phase**3) * p4[0,2]) + (35 * (1 - phase)**3 * (phase**4) * p5[0,2]) + (21 * (1 - phase)**2 * (phase**5) * p6[0,2]) + (7 * (1 - phase) * (phase**6) * p7[0,2]) + ((phase**7) * p8[0,2])
    
    
ori_data = np.array([.0, .0, .0])
gyr_data = np.array([.0, .0, .0])
com_data = np.array([.0, .0, .0])
zmp_data = np.array([.0, .0, .0])
acc_data = np.array([.0, .0, .0])

igl_data = 0.0

def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y-5*np.pi/180
    ori_data[2] = msg.z

def gyr_callback(msg):
    gyr_data[0] = msg.x
    gyr_data[1] = msg.y
    gyr_data[2] = msg.z

def com_callback(msg):
    com_data[0] = msg.x
    com_data[1] = msg.y
    com_data[2] = msg.z

def zmp_callback(msg):
    zmp_data[0] = msg.x
    zmp_data[1] = msg.y
    zmp_data[2] = msg.z

def acc_callback(msg):
    acc_data[0] = msg.x
    acc_data[1] = msg.y
    acc_data[2] = msg.z

# def yaw_callback(msg):
#     acc_data[0] = msg.x
#     acc_data[1] = msg.y
#     acc_data[2] = msg.z
    

igl_data_roll = 0.0

def main():
    global igl_data_roll
    rospy.init_node('bismillah', anonymous=False)
    rospy.Subscriber("com", Vector3, com_callback)
    rospy.Subscriber("zmp", Vector3, zmp_callback)
    rospy.Subscriber("acc", Vector3, acc_callback)
    # rospy.Subscriber("ori", Vector3, ori_callback)
    # rospy.Subscriber("gyr", Vector3, gyr_callback)
    # com_pub = rospy.Publisher('com_data', Vector3, queue_size=1)
    rate = rospy.Rate(30)

    # com_msg = Vector3()

    ik = InverseKinematic()
    sc = ServoController()
    
    rows = []
    
    AXIS  = np.array([-1,1,-1,-1,1,1, -1,-1,1,1,1,-1, 0,0,0, 0,0,0])
    COM   = np.matrix([0.0, 0.0, 0.23])
    LEFT  = np.matrix([0.0, 48 / 1000, 0.0])
    RIGHT = np.matrix([0.0, -48 / 1000, 0.0])
    phase = 0

    uCOM   = np.matrix([0.0, 0.0, 0.23])
    uLEFT  = np.matrix([0.0, 48 / 1000, 0.0])
    uRIGHT = np.matrix([0.0, -48 / 1000, 0.0])
    
    time_start = rospy.Time.now().to_sec()
    phase = 0.0
    state_time = np.array([7,2,2,2,2,2,1.7,2,2,2,1,4,4,4,2])#BELOK kanan

    state = 0
    acc_sebelumnyax = 0
    acc_sebelumnyay = 0
    vel_sebelumnyax = 0
    vel_sebelumnyay = 0
    
    while not rospy.is_shutdown():
        if phase >= 1:
            time_start = rospy.Time.now().to_sec()
            phase = 0
            
            if state == 14:
                break; 
            else :
                state = state + 1

            uCOM = COM
            uRIGHT = RIGHT
            uLEFT = LEFT
        else:
            time_now = rospy.Time.now().to_sec() - time_start
            phase = time_now / state_time[state]

        if state == 0:
            print("running")
            yaw_target1 = 0 #kiri
            yaw_target2 = 0 #kanan
            roll_target1 = 0
            roll_target2 = 0
            roll_target3 = 0
            roll_target4 = 0
            pitch_target1 = 0 
            pitch_target2 = 0 
            
            # pitch_target2 = -4 #  PITCH KANAN  - naik +turun (kanan)
            # pitch_target1 = -4 # PITCH KIRI  - turun +naik (kiri)
            # roll_target2 = 5 # ROLL KIRI (+ kanan - kiri)
            # roll_target1 = 5 # ROLL kanan (+ kiri - kanan)
            # yaw_target1 = 0 #kiri
            # yaw_target2 = 0 #kanan
           

# # ------------------------------------------------------ BELOK KANAN  ------------------------------------------------------------      
        # -------------------------- Langkah 1 -----------------------------
        if state == 1:
            yaw_target1 = 4
            roll_target1 = 2.3 #+ kanan
            pitch_target2 = -0.12
            roll_target2 = 0
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.035, -0.0417, 0.227]),np.matrix([0.03, 0.0, 0.227]))
            LEFT =   bezier_curve_4D(phase, uLEFT, np.matrix([0.00, 70 / 1000, 0.00]), np.matrix([0.040, 80 / 1000, 0.080]),np.matrix([0.045, 80 / 1000, 0.01]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.000, -48 / 1000, 0.00])) #-kiri
            
        # # # -------------------------- Langkah 2 -----------------------------    
        elif state == 2:
            roll_target1 = 3.1-3.1
            roll_target2 = 2 #+ kanan
            pitch_target2 = 0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.035, 0.01, 0.227]))
            LEFT =  bezier_curve_2D(phase, uLEFT,  np.matrix([0.055, 68.6 / 1000, 0.00])) #-kiri
          
        # # # -------------------------- Langkah 3 -----------------------------  
        elif state == 3:
            roll_target2 = 3+1.7 #+ kanan
            pitch_target1 = 2
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.045, 0.045 , 0.227]),np.matrix([0.03, 0.037, 0.23]))
            LEFT =  bezier_curve_2D(phase, uLEFT,  np.matrix([0.040, 76 / 1000, 0.00])) #-kiri
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.030, -70 / 1000, 0.04]))
      
        # # # # # #-------------------------- Langkah 4 -----------------------------
        elif state == 4:
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.0517, 0.033, 0.227]),np.matrix([0.04, -0.01, 0.227]))
            yaw_target2 = 15
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.060, -63 / 1000, 0.00]))   
             
        # # # # #-------------------------- Langkah 5 ----------------------------- 
        elif state == 5:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.035, 0.0, 0.227]))
            pitch_target2 = -0.5
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.000, -63 / 1000, 0.00]))
       
        # # # # #-------------------------- Langkah 6 -----------------------------    
        elif state == 6:
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.04, -0.065, 0.227]),np.matrix([0.065, -0.04, 0.227]))
            roll_target1 = 1 #+ kanan
            pitch_target1 = -1.5
            LEFT =  bezier_curve_3D(phase, uLEFT,  np.matrix([0.040, 81 / 1000, 0.04]),np.matrix([0.05, 81 / 1000, 0.06]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.000, -63 / 1000, 0.00]))
       
           
            # pitch_target2 = -4 #  PITCH KANAN  - naik +turun (kanan)
            # pitch_target1 = -4 # PITCH KIRI  - turun +naik (kiri)
            # roll_target2 = 5 # ROLL KIRI (+ kanan - kiri)
            # roll_target1 = 5 # ROLL kanan (+ kiri - kanan)
            
        # # # # #-------------------------- Langkah 7 -----------------------------    
        elif state == 7: 
            roll_target2 = 1
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.05, 0.01, 0.227]))
            yaw_target1 = 15
            pitch_target2 = -1.5
            LEFT =  bezier_curve_2D(phase, uLEFT,  np.matrix([0.075, 81 / 1000, 0.00]))
        
# # #         # # # # # #-------------------------- Langkah 8 -----------------------------    
        elif state == 8:
            pitch_target1 = 0
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.04, 0.0, 0.227]))
            
           # pitch_target2 = -4 #  PITCH KANAN  - naik +turun (kanan)
            # pitch_target1 = -4 # PITCH KIRI  - turun +naik (kiri)
            # roll_target2 = 5 # ROLL KIRI (+ kanan - kiri)
            # roll_target1 = 5 # ROLL kanan (+ kiri - kanan)
            
        elif state == 9: 
            roll_target2 = 3.5 #+ kanan
            pitch_target1 = 1
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.035, 0.047 , 0.227]),np.matrix([0.03, 0.0367 , 0.227]))
            LEFT =  bezier_curve_2D(phase, uLEFT,  np.matrix([0.060, 63 / 1000, 0.00])) #-kiri
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.040, -63 / 1000, 0.04]))
        
# #         # # # # # #  #-------------------------- Langkah 10 -----------------------------    
        elif state == 10: 
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.075, 0.05, 0.227]),np.matrix([0.045, -0.003, 0.227]))
            pitch_target1 = 3.5
            pitch_target2 = -3.9
            yaw_target1 = -3
            roll_target1 = 0.85#+ kanan
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.045, -48 / 1000, 0.00]))
              
#         # # # # #  #-------------------------- Langkah 11 -----------------------------    
        elif state == 11:
            COM = bezier_curve_2D(phase, uCOM, np.matrix([0.085, -0.0274, 0.227]))
            roll_target1 = 1.5#+ kanan
            # pitch_target2 = -2.5
            LEFT =  bezier_curve_2D(phase, uLEFT, np.matrix([0.075, 63 / 1000, 0.06]))
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.045, -48 / 1000, 0.0]))
            
        # # #-------------------------- Langkah 12 -----------------------------    
        elif state == 12: 
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.025, -0.026, 0.227]),np.matrix([0.075, 0.0 , 0.227]))
            yaw_target2 = -1.5
            # pitch_target2 = -1.5
            pitch_target1 = -1.15
            
            LEFT =  bezier_curve_2D(phase, uLEFT,  np.matrix([0.08, 78.7 / 1000, 0.0]))  
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.045, -48 / 1000, 0.0]))
            
# # # #         #     # pitch_target2 = -4 #  PITCH KANAN  - naik +turun (kanan)
# # # #         #     # pitch_target1 = -4 # PITCH KIRI  - turun +naik (kiri)
# # # #         #     # roll_target2 = 5 # ROLL KIRI (+ kanan - kiri)
# # # #         #     # roll_target1 = 5 # ROLL kanan (+ kiri - kanan)

# #         #    # # -------------------------- Langkah 13 -----------------------------  
        elif state == 13:
            roll_target2 = 3+1.7 #+ kanan
            pitch_target1 = 1
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.065, 0.045 , 0.227]),np.matrix([0.052, 0.035, 0.23]))
            LEFT =  bezier_curve_2D(phase, uLEFT,  np.matrix([0.040, 76 / 1000, 0.00])) #-kiri
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.030, -70 / 1000, 0.04]))
      
# #         # # # # -------------------------- Langkah 2 -----------------------------    
        elif state == 14:
            COM = bezier_curve_3D(phase, uCOM, np.matrix([0.0517, 0.033, 0.227]),np.matrix([0.055, 0.0, 0.227]))
            yaw_target2 = 0
            RIGHT =  bezier_curve_2D(phase, uRIGHT,  np.matrix([0.060, -63 / 1000, 0.00])) 
                   
        JOINTS = ik.solve(COM,LEFT, RIGHT)

        K = np.array([-0.04, -0.05])#gain proposional, gain derivatife (pitch)0.06, 0.01
        Kr = np.array([-0.085, -0.06])#gain proposional, gain derivatife (roll)0.06, 0.06    -0.065, -0.05
        Ky = np.array([-0.045, -0.05])
        igl_data_roll += ori_data[0]
       
        delta_pitch1 = K[0] * + (pitch_target1 + ori_data[1]) + K[1] * -gyr_data[1]
        delta_pitch2 = K[0] * + (pitch_target2 + ori_data[1]) + K[1] * -gyr_data[1]       
        delta_roll1 = Kr[0] * (roll_target1 + ori_data[0]) + Kr[1] * -gyr_data[0]
        delta_roll2 = Kr[0] * (roll_target2 + ori_data[0]) + Kr[1] * -gyr_data[0]
        delta_roll3 = Kr[0] * (roll_target3 + ori_data[0]) + Kr[1] * -gyr_data[0]
        delta_roll4 = Kr[0] * (roll_target4 + ori_data[0]) + Kr[1] * -gyr_data[0]
        delta_yaw1 = Ky[0] * (yaw_target1 - ori_data[2]) + Ky[1] * gyr_data[2] 
        delta_yaw2 = Ky[0] * (yaw_target2 - ori_data[2]) + Ky[1] * gyr_data[2] 
       
        # print((delta_roll * 180/np.pi))
        
      
        if (state>0):
            
            Kx = np.array([1.0000,    3.9310,    4.2263])
            Ky = np.array([0.3162,    1.4472,    1.7304])
            
            deltax = COM[0,0]*Kx[0] + (acc_data[0]*(time-rospy.Time.now().to_sec())*Kx[1])+vel_sebelumnyax + acc_data[0]*Kx[2]
            acc_robotx = deltax*(time-rospy.Time.now().to_sec())+acc_sebelumnyax
            vel_robotx = acc_robotx*(time-rospy.Time.now().to_sec())+vel_sebelumnyax
            pos_robotx = vel_robotx*(time-rospy.Time.now().to_sec())
            COM[0,0] += pos_robotx 
        
            deltay = COM[0,1]*Ky[0] + (acc_data[1]*(time-rospy.Time.now().to_sec())*Ky[1])+vel_sebelumnyay + acc_data[1]*Ky[2]
            acc_roboty = deltay*(time-rospy.Time.now().to_sec())+acc_sebelumnyay
            vel_roboty = acc_roboty*(time-rospy.Time.now().to_sec())+vel_sebelumnyay
            pos_roboty = vel_roboty*(time-rospy.Time.now().to_sec())
            COM[0,1] += pos_roboty 
        
        time = rospy.Time.now().to_sec()
        
        JOINTS[3] += delta_pitch1 #angkle kiri
        JOINTS[9] -= delta_pitch2 
        JOINTS[10] += delta_roll1 #1 kiri 2 kanan angkle kanan
        JOINTS[4] -= delta_roll2
        JOINTS[0] -= delta_roll3
        JOINTS[6] += delta_roll4
        #joint 7 = lutut kanan , 1 pitch hips kiri , 3 pitch angkle kiri, 4 roll angkle kiri, 0 roll hips kiri   
        JOINTS[5] += delta_yaw1 
        JOINTS[11] -= delta_yaw2 
        
        hand = [821,727,592, 207,300,435]

        JOINTS = np.array(np.hstack((JOINTS,hand)))

        sc.sync_write_pos(JOINTS*AXIS)

        acc_sebelumnyax = acc_data[0]
        vel_sebelumnyax = acc_sebelumnyax*(time-rospy.Time.now().to_sec())
        
        acc_sebelumnyay = acc_data[1]
        vel_sebelumnyay = acc_sebelumnyay*(time-rospy.Time.now().to_sec())
        
        rows.append([rospy.Time.now().to_sec(), com_data[0], com_data[1], com_data[2], zmp_data[0], zmp_data[1],  COM[0,0], COM[0,1], RIGHT[0,2], LEFT[0,2], yaw_target1, yaw_target2])

        # print(COM[0,0], COM[0,1], COM[0,2])
        
        rate.sleep()
    
    df = pd.DataFrame(rows, columns=['time','COM_X', 'COM_Y', 'COM_Z', 'ZMP_X', 'ZMP_Y', 'X_bez', 'Y_bez', 'R_bez', 'L_bez', 'L_Yaw','R_Yaw'])
   # Create a Pandas Excel writer using XlsxWriter as the engine.
    writer = pd.ExcelWriter('Fix_Kanan_2.xlsx', engine='xlsxwriter')

    # Convert the dataframe to an XlsxWriter Excel object.
    df.to_excel(writer, sheet_name='Sheet1', index=False)
    writer.close()

   
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
