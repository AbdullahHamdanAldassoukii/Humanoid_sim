import sys
import time
import numpy as np
import math as Math
import simulation.vrepSim.vrep as vrep
#spy#
# import vrep as vrep

NUM_OF_MOTORS = 18
MOTORS_PREFIX = "ART_"
BIOLOID_IN_VREP = 'BIOLOID'
FLOOR_IN_VREP = '5mx5mWoodenFloor'
IMU_IN_VREP = 'GyroSensor'
R_PROXIMITY = 'right_proximity'
L_PROXIMITY = 'left_proximity'

MODE = vrep.simx_opmode_oneshot_wait;
MODE_STREAMING = vrep.simx_opmode_streaming
MODE_BUFFER = vrep.simx_opmode_buffer
MODE_WAITE = vrep.simx_opmode_oneshot_wait
MODE_BLOCK = vrep.simx_opmode_blocking

g= np.transpose(np.array([0.0, 0.0, -9.8100004196167]))

# ############################################ POSTUREs ###############################################

INITIAL_POS = [336, 687, 298, 724, 412, 611, 355, 664, 491, 530, 394, 625, 278, 743, 616, 405, 490, 530]

STAND_POS = [512] * 18
STAND_POS[6] = 512 - 151
STAND_POS[7] = 512 + 151  # 151 = 45 degrees

# #####################################################################################################

class Bioloid:

    def __init__(self):

        # here we establish a connection to Vrep
        self.clientID = self.connect()

        # this list will contain the Motors IDs with size = 18 i.e ID[3]= id for motor 4

        # get the Robot Objects Handles (Handlers)
        self.ID, self.robot, self.floor, self.IMU, self.prox_R, self.prox_L = self.get_handles()

        # this var is the time that would be the para of the path equations
        self.t = 1.0

        self.init_mode()
        pass

    def get_handles(self):

        ID = [0] * NUM_OF_MOTORS
        # get Motors Handles
        for i in range(NUM_OF_MOTORS):
            motor_name = MOTORS_PREFIX + str(i + 1)
            errcode, ID[i] = vrep.simxGetObjectHandle(self.clientID, motor_name, MODE)
            # errcode= 0 means The function executed fine
            if errcode == 0:
                print("Motor (" + str(i + 1) + ") connected !")
            else:
                print("Warning :Motor (" + str(i + 1) + ") is not connected !")

        # Robot Handle
        errcode, robot = vrep.simxGetObjectHandle(self.clientID, BIOLOID_IN_VREP, MODE)

        if errcode == 0:
            print("Robot Connected !")
        else:
            print("Warning : Robot disconnected !")

        # floor handle

        errcode, floor = vrep.simxGetObjectHandle(self.clientID, FLOOR_IN_VREP, MODE)

        if errcode == 0:
            print("floor Connected !")
        else:
            print("Warning : floor disconnected !")

        # # IMU Handle TODO
        # self.IMU = self.get_IMU_handle()
        errcode, imu = vrep.simxGetObjectHandle(self.clientID, IMU_IN_VREP, MODE)
        if errcode == 0:
            print("IMU Connected !")
        else:
            print("Warning : IMU disconnected !")

        errcode, proximity_R = vrep.simxGetObjectHandle(self.clientID, R_PROXIMITY, MODE)
        if errcode == 0:
            print("right proximity sensor Connected !")
        else:
            print("Warning : right proximity sensor disconnected !")
        errcode, proximity_L = vrep.simxGetObjectHandle(self.clientID, L_PROXIMITY, MODE)
        if errcode == 0:
            print("left proximity sensor Connected !")
        else:
            print("Warning : left proximity sensor disconnected !")

        return ID, robot, floor, imu, proximity_R, proximity_L

        # def get_IMU_handle(self):

    def read_imu_data(self):
        start_time = time.time()
        vrep.simxCallScriptFunction(self.clientID, 'GyroSensor', vrep.sim_scripttype_childscript, 'getGyroData',
                                    [], [], [], bytearray(3), MODE_STREAMING)

        while time.time() - start_time < 0.028:
            ret, intDataOut, floatDataOut, stringDataOut, bufferOut = vrep.simxCallScriptFunction(self.clientID,
                                                                                                  'GyroSensor',
                                                                                                  vrep.sim_scripttype_childscript,
                                                                                                  'getGyroData', [], [],
                                                                                                  [], bytearray(3),
                                                                                                  MODE_BUFFER)
            errcode = ret
            imu_data = floatDataOut

            # After initialization of streaming, it will take a few ms before the first value arrives, so check the
            # return code
            if errcode == vrep.simx_return_ok:
                print("Gyro data: [r p y] = " + str(imu_data))
                return imu_data

        pass

    # this IMU what we will use in RL
    def read_imu_data_MPU(self):
        errcode, linear_vel, angular_vel = vrep.simxGetObjectVelocity(self.clientID, self.IMU, MODE_BUFFER)
        return linear_vel, angular_vel

    def get_gravity_vector(self):
        errCode,euler_angles=vrep.simxGetObjectOrientation(self.clientID,
                                                           self.IMU,
                                                           self.floor,
                                                           MODE_BUFFER)

        roll,pitch,yaw = euler_angles
        R = np.array([[Math.cos(pitch) * Math.cos(yaw),
             Math.sin(pitch) * Math.sin(roll) * Math.cos(yaw) - Math.sin(yaw) * Math.cos(roll),
             Math.sin(pitch) * Math.cos(roll) * Math.cos(yaw) + Math.sin(roll) * Math.sin(yaw)],
            [Math.sin(yaw) * Math.cos(pitch),
             Math.sin(pitch) * Math.sin(roll) * Math.sin(yaw) + Math.cos(roll) * Math.cos(yaw),
             Math.sin(pitch) * Math.sin(yaw) * Math.cos(roll) - Math.sin(roll) * Math.cos(yaw)],
            [-Math.sin(pitch), Math.sin(roll) * Math.cos(pitch), Math.cos(pitch) * Math.cos(roll)]
        ])
        gravity= np.matmul(g,R)
        return gravity

    # this function will return Boolean values of the right and left touch sensor
    # return: True  => in touch
    #         False => not in touch
    def read_touch_sensor(self):
        e, f, r, fd, ffd = vrep.simxReadProximitySensor(self.clientID, self.prox_R, MODE_BUFFER)
        e, f, l, fd, ffd = vrep.simxReadProximitySensor(self.clientID, self.prox_L, MODE_BUFFER)
        limit = 0.0087
        right = r[2] >= limit  # if true =>  not touch the ground
        left = l[2] >= limit  # if true =>  not touch the ground
        
        #print("left="+str(l[2])+"     right = "+str(r[2]))

        return int(not right), int(not left)

    def read_joints_velocity(self):
        vel = [0] * NUM_OF_MOTORS
        velocity_parameter_id = 2012  # according to vrep http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        for i in range(NUM_OF_MOTORS):
            _, vel[i] = vrep.simxGetObjectFloatParameter(self.clientID, self.ID[i], velocity_parameter_id,
                                                         MODE_BUFFER  # could be MODE_BLOCK
                                                         )

        return vel

    def read_joints_position(self):
        pos = [0] * NUM_OF_MOTORS
        for i in range(NUM_OF_MOTORS):
            errCode, pos[i] = vrep.simxGetJointPosition(self.clientID, self.ID[i], MODE_BUFFER)

        return pos

    def get_state_vector(self):
        g = self.get_gravity_vector()  # this is 3-vector
        linear_vel, angular_vel = self.read_imu_data_MPU()  # this is 6-vector
        q = self.read_joints_position()  # this is 18-vector
        qd = self.read_joints_velocity()  # this is 18-vector
        tc = self.read_touch_sensor()  # this is 2-vector

        state = np.r_[np.r_[np.r_[np.r_[np.r_[g, linear_vel], angular_vel], q], qd], tc]

        state_list = [g, linear_vel, angular_vel, q, qd, tc]
        return np.array([state]), state_list

    def read_cm(self):
        errCode, pos = vrep.simxGetObjectPosition(self.clientID, self.robot, self.floor, MODE_STREAMING)
        print("CM Position : [ x , y , z ]= " + str(pos))
        return pos

    def set_degree(self, connection_time, pos, mode='dynamexil'):
        c = 0
        if mode=='dynamexil':
            while (self.clientID != -1) & (c != connection_time):
                for i in range(0, NUM_OF_MOTORS):
                    vrep.simxSetJointTargetPosition(self.clientID, self.ID[i], self.dynamexil2rad(pos[i]),
                                                    MODE_STREAMING)

                c += 1
        elif mode=='rad':
            while (self.clientID != -1) & (c != connection_time):
                for i in range(0, NUM_OF_MOTORS):
                    vrep.simxSetJointTargetPosition(self.clientID, self.ID[i], pos[i],MODE_STREAMING)

                c += 1


        # TODO play with this value

        time.sleep(0.03)

        pass

    # this function convert the angle in degree to Dynamexil Rang
    def rad2dynamixel(self, degree):
        dy = (degree + 2.62) / 0.00511711875
        return dy

    # this funcion convert from Motor Rang i.e [0 , 1023] to degree
    def dynamexil2rad(self, dy):
        degree = -2.62 + (dy * 0.00511711875)
        return degree

    # this method return the current value of robot motors / specific motor with an id
    # id is zero based indexing
    def read_motor(self, id=None):

        if id:
            return vrep.simxGetJointPosition(self.clientID, self.ID[id], MODE_BUFFER)

        motor_pos = [0.0] * NUM_OF_MOTORS

        for i in range(0, NUM_OF_MOTORS):
            errCode, motor_pos[i] = vrep.simxGetJointPosition(self.clientID, self.ID[i], MODE_BUFFER)
            print("Motor " + str(i + 1) + " = " + str(self.rad2dynamixel(motor_pos[i])))

        return motor_pos

    # this method return if the robot has fall or not
    # return : Boolean
    def isFall(self, state_list):
        gx, gy, gz = state_list[0]
        gmax = np.sqrt(9.8100004196 / 2)
        x = np.linalg.norm([gy, gx])
        is_fall = (x - gmax) >= 0
        return is_fall

    def get_reward(self, state_list):

        gx, gy, gz = state_list[0]
        V = state_list[1]
        vx, vy, vz = V
        W = state_list[2]
        wx, wy, wz = W
        tc = state_list[5]

        a0 = 0.2
        a1 = 0.001
        a2 = 40
        a3 = 0.02
        a4 = 0.0003
        a5 = 0.015
        gmax = np.sqrt(9.8100004196 / 2)

        x = np.linalg.norm([gx, gy])

        speed_reward = a0 * vx
        oscillations_penalty = - a1 * wy
        falling_penalty = -a2 * max(x - gmax, 0)
        tilting_penalty = -a3 * x
        slip_rotation_penalty = - a4 * wz
        one_foot_support_reawrd = a5 * abs(tc[0] - tc[1])

        reward = speed_reward + \
                 oscillations_penalty + \
                 falling_penalty + \
                 tilting_penalty + \
                 slip_rotation_penalty + \
                 one_foot_support_reawrd
        return reward

    # this method return the action according to the inverse prediction model IPM
    def ipm(self):
        increment = 0.05
        IPM = [0] * 18
        i=self.t

        IPM[0] = 333.5
        IPM[1] = 690
        IPM[2] = 297.8
        IPM[3] = 724
        IPM[4] = 412
        IPM[5] = 611.2
        IPM[6] = 355
        IPM[7] = 664
        IPM[8] = -0.000275921606029018 * Math.pow(i, 6) + 0.0224866058355711 * Math.pow(i, 5) - 0.554002659792176 * Math.pow(i, 4) + 5.27359561063168 * Math.pow(i, 3) - 16.6250714681205 * Math.pow(i, 2) + 1.07731363098685 * i + 492.577239841013
        IPM[9] = -0.000140124106979082 * Math.pow(i, 6) + 0.00957747098059501 * Math.pow(i, 5) - 0.17601617035749 * Math.pow(i, 4) + 0.366976848153587 * Math.pow(i, 3) + 13.7934195723444 * Math.pow(i, 2) - 82.4753256531787 * i + 597.355314009854;
        IPM[10] = 0.00000140489885303541 * Math.pow(i, 12) - 0.000137098037479588 * Math.pow(i, 11) + 0.00593962266650624 * Math.pow(i, 10) - 0.150467310047963 * Math.pow(i, 9) + 2.47028882826084 * Math.pow(i, 8) - 27.5207922276621 * Math.pow(i, 7) + 211.699439008591 * Math.pow(i, 6) - 1121.79501433143 * Math.pow(i, 5) + 4012.95422656784 * Math.pow(i, 4) - 9303.65422686671 * Math.pow(i, 3) + 13044.0770540131 * Math.pow(i, 2) - 9823.0075743571 * i + 3401.38059312374;
        IPM[11] = 0.00000011672084327608 * Math.pow(i, 14) - 0.000012662080312946 * Math.pow(i, 13) + 0.000615851466539234 * Math.pow(i, 12) - 0.017737457506048 * Math.pow(i, 11) + 0.336649535916704 * Math.pow(i, 10) - 4.43504476680317 * Math.pow(i, 9) + 41.6304852819428 * Math.pow(i, 8) - 281.48687768283 * Math.pow(i, 7) + 1370.97665082275 * Math.pow(i, 6) - 4763.85605618472 * Math.pow(i, 5) + 11574.9816970237 * Math.pow(i, 4) - 18997.7828225834 * Math.pow(i, 3) + 19842.4241425091 * Math.pow(i, 2) - 11754.8617513084 * i + 3597.71125928176;
        IPM[12] = -0.0000148141287899076 * Math.pow(i, 10) + 0.00119700195674767 * Math.pow(i, 9) - 0.0415853177723568 * Math.pow(i, 8) + 0.813243772863493 * Math.pow(i, 7) - 9.8518102730438 * Math.pow(i, 6) + 76.7204688187178 * Math.pow(i, 5) - 385.724100593995 * Math.pow(i, 4) + 1221.58291298422 * Math.pow(i, 3) - 2295.15666871898 * Math.pow(i, 2) + 2304.35552800843 * i - 643.058578051134;
        IPM[13] = 0.00000083846544852191 * Math.pow(i, 10) - 0.00000812251364211665 * Math.pow(i, 9) - 0.00194976423616432 * Math.pow(i, 8) + 0.0861933684303852 * Math.pow(i, 7) - 1.69603346375747 * Math.pow(i, 6) + 18.9297051399361 * Math.pow(i, 5) - 127.429816133823 * Math.pow(i, 4) + 517.304913254853 * Math.pow(i, 3) - 1215.69884225685 * Math.pow(i, 2) + 1501.00777573452 * i + 34.6214069610031;
        IPM[14] = 0.0000217898674376092 * Math.pow(i, 10) - 0.00168504911464163 * Math.pow(i, 9) + 0.0556171425516435 * Math.pow(i, 8) - 1.02212510842425 * Math.pow(i, 7) + 11.4361911769592 * Math.pow(i, 6) - 79.9218400189809 * Math.pow(i, 5) + 343.33892677256 * Math.pow(i, 4) - 852.044744171629 * Math.pow(i, 3) + 1066.9698822212 * Math.pow(i, 2) - 535.985198422995 * i + 651.394169313619;
        IPM[15] = 0.0000248742798796263 * Math.pow(i, 10) - 0.00191083364372945 * Math.pow(i, 9) + 0.0626362655558014 * Math.pow(i, 8) - 1.1448672116933 * Math.pow(i, 7) + 12.8113562500397 * Math.pow(i, 6) - 90.8916292359355 * Math.pow(i, 5) + 410.536161792987 * Math.pow(i, 4) - 1157.63710507665 * Math.pow(i, 3) + 1944.09966170448 * Math.pow(i, 2) - 1764.8116655254 * i + 1056.68768177018;
        IPM[16] = -0.000602821242505626 * Math.pow(i, 6) + 0.0405315649727053 * Math.pow(i, 5) - 0.916931723020958 * Math.pow(i, 4) + 8.3576698861698 * Math.pow(i, 3) - 25.507120415259 * Math.pow(i, 2) - 1.29495830014807 * i + 497.088587623408;
        IPM[17] = -0.00021958932773343 * Math.pow(i, 6) + 0.0135980370756124 * Math.pow(i, 5) - 0.239784506804081 * Math.pow(i, 4) + 0.466436883005924 * Math.pow(i, 3) + 18.9979103475509 * Math.pow(i, 2) - 113.421209371814 * i + 620.07665583061;

        # TIME_LIMIT=15.7
        TIME_LIMIT = 14.7

        if self.t >= TIME_LIMIT:
            self.t = 2.1
        else:
            self.t += increment
        return np.array(IPM)

    def init_mode(self):
        velocity_parameter_id = 2012  # according to vrep http://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        for i in range(0, NUM_OF_MOTORS):
            vrep.simxGetJointPosition(self.clientID, self.ID[i], MODE_STREAMING)
            vrep.simxGetObjectFloatParameter(self.clientID, self.ID[i], velocity_parameter_id,
                                             MODE_STREAMING  # could be MODE_BLOCK
                                             )

        vrep.simxGetObjectVelocity(self.clientID, self.IMU, MODE_STREAMING)
        vrep.simxReadProximitySensor(self.clientID, self.prox_R, MODE_STREAMING)
        vrep.simxReadProximitySensor(self.clientID, self.prox_L, MODE_STREAMING)
        vrep.simxGetArrayParameter(self.clientID, vrep.sim_arrayparam_gravity, MODE_STREAMING)
        vrep.simxGetObjectOrientation(self.clientID,self.IMU,self.floor,MODE_STREAMING)
        pass

    def restart(self):
        # vrep.simxRemoveModel (self.clientID,self.robot,vrep.simx_opmode_blocking)
        # vrep.simxLoadModel(self.clientID,"C:\\Program Files\\V-REP3\\V-REP_PRO_EDU\\scenes\\Bioloid.ttt", 1,vrep.simx_opmode_blocking)
        self.t=1.0
        vrep.simxStopSimulation(self.clientID, MODE_BLOCK)
        time.sleep(6.0)
        a = vrep.simxStartSimulation(self.clientID, MODE_BLOCK)
        self.set_degree(1,STAND_POS)
        time.sleep(3.0)
        self.set_degree(1, INITIAL_POS)
        time.sleep(3.0)
        return a

    def clear(self):
        vrep.simxStopSimulation(self.clientID, MODE_BLOCK)
        time.sleep(5.0)
        pass

    @staticmethod
    def connect():
        vrep.simxFinish(-1)  # just in case, close all opened connections
        # NOTE : the port was 19999 but with continuous remote API we change it to 19998 after editing remoteApiConnections.txt
        # refers: http://www.forum.coppeliarobotics.com/viewtopic.php?t=537

        clientID = vrep.simxStart('127.0.0.1', 19998, True, True, 5000, 5)  # Connect to V-REP
        if clientID != -1:
            print("Connection succeeded!")
        else:
            print("Connection Failed!")
            sys.exit("Cannot connect!")

        return clientID

    @staticmethod
    def disconnect(self):
        # TODO
        pass

if __name__ == "__main__":
    Bioloid()
