import numpy as np 
from matplotlib import pyplot as plt 
from matplotlib import animation 
import time 
from nrf24 import NRF24 
import math
import serial
from sympy import Symbol, solve, Float, I, sympify
import threading

class plot(threading.Thread):
    def run(self):
        plt.legend()
        plt.grid(True)
        plt.show()

class Turtle:
    def __init__(self):
        self.ser = serial.Serial('/dev/serial0', 9600)
        self.r1 = 1.0
        self.r2 = 1.0
        self.r3 = 1.0
        self.r4 = 1.0
        self.Mx = 1.0
        self.My = 1.0
        self.Mz = 1.0
        self.Px = 1.0
        self.Py = 1.0
        self.Pz = 1.0

        self.Cx = 8
        self.Cy = 8
        self.ZZ = 2
        self.tutlePo = [5.38,0,0.2]
        self.RR = 1
        self.Ry = 1
        self.Rx = 1
        self.Rz = 1
        self.Ab = 0
        self.spiout = []
        self.smode = '0'
        self.Az = 0.54
    def SerialSend(self, mode):
        if mode == 1:
            send = self.spiout
        elif mode == 2:
            send = '@@'
        self.ser.write(send.encode())
        
    def isNumber(self, s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def Calcula(self):
        self.Mx = (self.Cx / 2) + ((self.r1**2 - self.r2**2 - self.r3**2 + self.r4**2) / (4 * self.Cx))
        self.My = (self.Cy / 2) + ((self.r1**2 + self.r2**2 - self.r3**2 - self.r4**2) / (4 * self.Cy)) +0.15
        self.Mz = 1.25
        if self.smode == '1':
            self.Az = 0
        elif self.smode == '0':
            self.Az = 0.54
        print('x: ', self.Mx, 'y: ', self.My)#'z: ', self.Mz)
        print('mode', self.smode)
        self.RR = (self.Mz-self.Az) / np.sin(self.Rz) 
        self.Px = np.cos(self.Ry)*self.RR*np.cos(self.Rz) + self.Mx
        self.Py = np.sin(self.Ry)*self.RR*np.cos(self.Rz) + self.My
        self.Pz = self.Mz - self.RR*np.sin(self.Rz)#np.sin(self.Rz) + self.ZZ - self.Mz
        if self.smode == '2':
            self.Px = 5.77
            self.Py = 2.27
            self.Pz = 0.54
        if self.smode == '3':
            self.Px = 4.2310032
            self.Py = 4.2510543
            self.Pz = 0.10
      
        print('Px: ', self.Px, 'Py: ', self.Py)
    def calVelo(self):
        r = math.sqrt((self.Px - self.tutlePo[0])**2 + (self.Py-self.tutlePo[1])**2)
        V = math.sqrt(9.8*(r + (self.Pz - self.tutlePo[2]) + (self.Pz - self.tutlePo[2])**2 / (r + (self.Pz - self.tutlePo[2]))))
        X_axi = math.atan2((self.Py-self.tutlePo[1]), (self.Px-self.tutlePo[0]))/3.141592*180
        Y_axi = math.atan2((r + 2*(self.Pz - self.tutlePo[2])), r) / 3.141592 * 180
        self.spiout = '#' + str(round(X_axi, 2)) + ','+str(round(Y_axi, 2))+',' + str(round(V*1000,2)) +';'
       #print(self.spiout)

def find_meetpoint(a, b, c, d, r1, r2):  #(x-a)^2 + (y-b)^2 = r1^2, (x-c)^2 + (y-d)^2 = r2^2
    def find_meetpointX(y1, y2):
        x = Symbol('x')

        eq1 = (x - a)**2 + (y1 - b)**2 - r1**2
        eq2 = (x - c)**2 + (y1 - d)**2 - r2**2
        eq3 = (x - a)**2 + (y2 - b)**2 - r1**2
        eq4 = (x - c)**2 + (y2 - d)**2 - r2**2
        result1 = []
        result2 = []
        sol1 = np.array(solve(eq1))
        sol2 = np.array(solve(eq2))
        sol3 = np.array(solve(eq3))
        sol4 = np.array(solve(eq4))
        for i in range(len(sol1)):
            sol1[i] = float(int(sol1[i] * 10000) / 10000)    # 소수점 넷째자리 아래 버림
            sol2[i] = float(int(sol2[i] * 10000) / 10000)
            sol3[i] = float(int(sol3[i] * 10000) / 10000)
            sol4[i] = float(int(sol4[i] * 10000) / 10000)

        for i in range(2):
            sol1[i] = Float(sol1[i])
            sol2[i] = Float(sol2[i])
            sol3[i] = Float(sol3[i])
            sol4[i] = Float(sol4[i])

        #print([sol1, sol2], [sol3, sol4])
        if sol1[0] in sol2:
            result1 = sol1[0]
        elif sol1[1] in sol2:
            result1 = sol1[1]
        else:
            print("EEROR, can't find meetpointX")
        if sol3[0] in sol4:
            result2 = sol3[0]
        elif sol3[1] in sol4:
            result2 = sol3[1]
        else:
            print("EEROR, can't find meetpointX")
        result = [result1, result2]
        #print(result)

        return result

    def find_meetpointY(x1, x2):
        y = Symbol('y')

        eq1 = (x1 - a)**2 + (y - b)**2 - r1**2
        eq2 = (x1 - c)**2 + (y - d)**2 - r2**2
        eq3 = (x2 - a)**2 + (y - b)**2 - r1**2
        eq4 = (x2 - c)**2 + (y - d)**2 - r2**2
        result1 = []
        result2 = []
        sol1 = np.array(solve(eq1))
        sol2 = np.array(solve(eq2))
        sol3 = np.array(solve(eq3))
        sol4 = np.array(solve(eq4))
        for i in range(len(sol1)):
            sol1[i] = float(int(sol1[i] * 10000) / 10000)    # 소수점 넷째자리 아래 버림
            sol2[i] = float(int(sol2[i] * 10000) / 10000)
            sol3[i] = float(int(sol3[i] * 10000) / 10000)
            sol4[i] = float(int(sol4[i] * 10000) / 10000)
        for i in range(2):
            sol1[i] = Float(sol1[i])
            sol2[i] = Float(sol2[i])
            sol3[i] = Float(sol3[i])
            sol4[i] = Float(sol4[i])

        #print(sol1, type(sol1))
        #print(Float(sol1[0]), type(Float(sol1[0])))

        if sol1[0] in sol2:
            result1 = sol1[0]
        elif sol1[1] in sol2:
            result1 = sol1[1]
        else:
            print("EEROR, can't find meetpointY")
        if sol3[0] in sol4:
            result2 = sol3[0]
        elif sol3[1] in sol4:
            result2 = sol3[1]
        else:
            print("EEROR, can't find meetpointY")
        result = [result1, result2]
        #print(result)

        return result

    x = Symbol('x')
    y = Symbol('y')

    if((b == d)):
        eqX = ((b-d)/(c-a)) * y + ((r1 ** 2 - r2 ** 2 - a ** 2 + c ** 2 - b ** 2 + d ** 2) / (2*(c-a)))
        eq2 = (eqX - a) ** 2 + (y - b) ** 2 - r1 ** 2
        resultY = np.array(solve(eq2))
        if resultY[0].is_real and resultY[1].is_real:       # 만나지 않는 원의 경우 결과값이 허수로 나옴 (예외처리)
            resultX = find_meetpointX(resultY[0], resultY[1])
            result = [[resultX[0], resultY[0]], [resultX[1], resultY[1]]]
        else:
            result = None

    elif(b is not d):
        eqY = ((c - a) / (b - d)) * x + ((r1 ** 2 - r2 ** 2 - a ** 2 + c ** 2 - b ** 2 + d ** 2) / (-2 * (b - d)))
        eq2 = (x - a) ** 2 + (eqY - b) ** 2 - r1 ** 2
        resultX = np.array(solve(eq2))
        if resultX[0].is_real and resultX[1].is_real:       # 만나지 않는 원의 경우 결과값이 허수로 나옴 (예외처리)
            resultY = find_meetpointY(resultX[0], resultX[1])
            result = [[resultX[0], resultY[0]], [resultX[1], resultY[1]]]
        else:
            result = None
    else:
        print("EEROR : gradient")

    return result

def remove_point(circle_num, center, radi):       ## 교점중 거리가 먼 점을 하나씩 지우는 함수
    result = []
    for i in range(circle_num - 1):      ## 교점찾는 부분
        for j in range(i + 1, circle_num, 1):
            meetpoint = find_meetpoint(center[i][0], center[i][1],
                                         center[j][0], center[j][1], radi[i], radi[j])
            if meetpoint is not None:
                result.append(meetpoint)
                print(result[len(result) - 1])
            else:
                print('Circle ' + str(i) + ' and Circle ' + str(j) + ' has no meeting point!')

    result = sum(result, [])        # list 차원 줄이기

    if len(result) is not 0:        ## 가까운 점 뽑는 부분
        while len(result) > 5:      # 점을 뽑을 개수
            pointX = 0
            pointY = 0
            for i in range(len(result)):
                plt.scatter(result[i][0], result[i][1], color='gold', s=10, zorder=4)
                pointX = pointX + result[i][0]
                pointY = pointY + result[i][1]

            est_position = [pointX/len(result), pointY/len(result)]
            p2p_distance = []
            for i in range(len(result)):
                p2p_distance.append(math.sqrt((result[i][0] - est_position[0])**2 + (result[i][1] - est_position[1])**2))   #평균 점과의 거리 구하기

            longest_point = result[p2p_distance.index(max(p2p_distance))]
            # print("longest distance is : " + str(max(p2p_distance)))
            # print("longest point is : " + str(longest_point))
            # print("est_position is : " + str(est_position))

            ## 이미지로 저장하려면 주석 해제 !! ##

            # plt.scatter(longest_point[0], longest_point[1], color='deeppink', s=10, zorder=5)
            # plt.scatter(est_position[0], est_position[1], color='red', label='est_position', s=15, zorder=6)
            #
            # a.set_aspect('equal')
            # plt.grid(True)
            # plt.savefig('./plot/fig' + str(len(result)) + '.png', dpi=300)


            del result[p2p_distance.index(max(p2p_distance))]     #거리가 가장 긴 점을 삭제
    elif len(result) is 0:
        print("ERROR : inside Point is none")
    return result


def Final_est_position(circle_num, circle_center, circle_radi):       ## 최종 교점을 평균내서 예측위치 찾는 함수
    last_point = remove_point(circle_num, circle_center, circle_radi)
    pointX = 0
    pointY = 0
    for i in range(len(last_point)):
        plt.scatter(last_point[i][0], last_point[i][1], color='darkblue', s=25, zorder=5)
        pointX = pointX + last_point[i][0]
        pointY = pointY + last_point[i][1]

    est_position = [pointX / len(last_point), pointY / len(last_point)]
    print('Final est_position is : ' + str(est_position))
    return est_position

def error_correction(distance):
    for i in range(len(distance)):
        distance[i] = -0.0812 * distance[i] + 0.4335 + distance[i]    #오차 보정식
        
    return distance
    
def Z_estimation(r_num, Anchor_pos, Anchor_dist, position):
    Z_est = []
    Z_AVG = 0.0;
    st_line = 0.0
    for i in range(r_num):
        st_line = ((position[0] - Anchor_pos[i][0])**2 + (position[1] - Anchor_pos[i][1])**2) ##z 거리 구할때 Anchor 거리보다 Anchor 직선 거리가 끌때 오류 처리
        if st_line > Anchor_dist[i]**2 :
            st_line = Anchor_dist[i]**2
            
        Z_est.append(Anchor_pos[i][2] - math.sqrt(Anchor_dist[i]**2 - st_line))
        Z_AVG = Z_AVG + Z_est[i]
    Z_AVG = Z_AVG / r_num
    print("Z-range : " + str(min(Z_est)) + " ~ " + str(max(Z_est)))
    print(str(Z_est[0]) + ' ' + str(Z_est[1]) + ' ' + str(Z_est[2]) + ' ' + str(Z_est[3]))
    print("AVG Z : " + str(Z_AVG))
    return Z_AVG

def target_est_pos(tag_pos, lidar, mpu):
    print('MPU : ' + str(mpu[0]) + ', ' + str(mpu[1]))
    print('TAG : ' + str(tag_pos[0]) + ', ' + str(tag_pos[1]) + ', ' + str(tag_pos[2]))
    print('Lidar : ' + str(lidar))
    target_pos = []
    target_x = tag_pos[0] + (lidar * math.cos(mpu[0]/(180.0/math.pi))) * math.cos(mpu[1]/(180.0/math.pi))
    target_y = tag_pos[1] + (lidar * math.cos(mpu[0]/(180.0/math.pi))) * math.sin(mpu[1]/(180.0/math.pi))
    target_z = tag_pos[2] - lidar * math.sin(mpu[0]/(180.0/math.pi))

    target_pos.append(target_x)
    target_pos.append(target_y)
    target_pos.append(target_z)
    
    print('목표물 위치 - X: ' + str(target_pos[0]) + ', Y: ' + str(target_pos[1]) + ', Z: ' + str(target_pos[2]))
    
    return target_pos

def lidar_error_correction(lidar, mpu):
    lidar_correction = lidar - (2 * math.tan((90 - mpu[0])/(180.0/math.pi))) * 0.01
    lidar_correction = lidar_correction + 0.8
    print('Lidar : ' + str(lidar) + '    오차보정 : ' + str(lidar_correction))
    return lidar_correction

def kkobuk_angle_calc(target_pos, kkobuk_pos):
    servo_x = 0.0
    servo_y = 0.0
    water_speed = 0.0
    
    x_distance = target_pos[0] - kkobuk_pos[0]
    y_distance = target_pos[1] - kkobuk_pos[1]
    
    side_line = 0.0

    if(x_distance > 0):
        servo_x = math.atan(y_distance / x_distance)*(180.0/math.pi)
        servo_x = float(int(servo_x * 100)/100)
        
    elif(x_distance <0):
        servo_x = math.atan(y_distance / x_distance)*(180.0/math.pi) + 180
        servo_x = float(int(servo_x * 100)/100)
    
    else:
        print('kkobuk_angle_calc ERROR!! maybe x_distance is zero')
        servo_x = 90.00
        
    side_line = math.sqrt((target_pos[0] - kkobuk_pos[0])**2 + (target_pos[1] - kkobuk_pos[1])**2 + (target_pos[2] - kkobuk_pos[2])**2)
    
    servo_y = math.atan((side_line + (2 * height)) / side_line)*(180.0/math.pi)
    servo_y = float(int(servo_y * 100)/100)
    
    #v = sqrt(중력 * (빗변 + 사출구 높이 + (사출구높이^2 / (빗변 + 사출구높이))))
    water_speed = math.sqrt(9.8 * (side_line + height + ((height**2) / (side_line + height)))) * 1000
    water_speed = int(water_speed)
    return servo_x, servo_y, water_speed

def SerialSend(data):
    
    def SerialSend(self, mode):
        if mode == 1:
            send = self.spiout
        elif mode == 2:
            send = '@@'
        self.ser.write(send.encode())
    
tu = Turtle()
b = plot()

print(tu.r1)
i=0
pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xff, 0xff, 0xff, 0xff, 0xff]]
radio = NRF24() 
radio.begin(0, 0, 8, 25) 
radio.setPayloadSize(0x32) 
radio.setChannel(0x6C) 
radio.setDataRate(NRF24.BR_1MBPS) 
radio.setPALevel(NRF24.PA_MAX) 
radio.openWritingPipe(pipes[1]) 
radio.openReadingPipe(0, pipes[0]) 
radio.startListening() 
radio.stopListening() 
radio.printDetails() 
radio.startListening()


color = ['black', 'gray', 'rosybrown', 'red', 'lightsalmon', 'darkorange',
         'gold', 'yellow', 'greenyellow', 'lightgreen', 'green', 'turquoise', 'aqua', 'deepskyblue']

# 원 예시
#   -11.5   2       8
#   2.5     3.5     11.5
#   4.5     -8.5    13
#   -7.5    -11     11.5
#   -3.5    -15     12.5

#r_num = int(input('원 개수 : '))     # 입력받을려면 주석 해제
#circle_center = []
# 입력받을시 주석 처리!
r_num = 4
#circle_center = [[0, 0], [0, 1.69], [2.98, 1.69], [2.98, 0]]
circle_center = [[0, 0, 1.8], [0, 8, 1.8], [8, 8, 1.8], [8, 0, 1.8]]
x_range = [None, None]
y_range = [None, None]
#kkobuk_pos = []
kkobuk_pos = [4.31, 0, 0]

ser = serial.Serial('/dev/serial0', 9600)

#YAW_cali = 0.0
YAW_cali = -13.58

for i in range(r_num):
    ## 입력받을시 주석 해제 !!
    #x, y = input('circle' + str(i) + ' center : ').split()      #각 원의 x, y좌표 입력
    #x = float(x)
    #y = float(y)
    #circle_center.append([x, y])

    #x, y = input('꼬북이 위치 - X_Y').split()   #꼬북이 위치 입력받음 (X Y)
    #x = float(x)
    #y = float(y)
    #kkobuk_pos.append([x, y])
    
    if x_range[0] is None:
        x_range[0] = circle_center[i][0]
        x_range[1] = circle_center[i][0]
    elif x_range is not None:
        if circle_center[i][0] < x_range[0]:
            x_range[0] = circle_center[i][0]
        elif circle_center[i][0] > x_range[1]:
            x_range[1] = circle_center[i][0]
    if y_range[0] is None:
        y_range[0] = circle_center[i][1]
        y_range[1] = circle_center[i][1]
    elif y_range is not None:
        if circle_center[i][1] < y_range[0]:
            y_range[0] = circle_center[i][1]
        elif circle_center[i][1] > y_range[1]:
            y_range[1] = circle_center[i][1]
            
while 1:
    num = np.linspace(0, 2 * np.pi, 100)
    cyclex = np.sin(num)
    cycley = np.cos(num)
    
    a = plt.axes()
    
    smode = 1
    endc = 1

    pipe = [0]
    radius = []
    split_data = []
    line = ''

    for i in range(r_num):
        radius.append(float(1))
    while 1:

        if smode == 1:
            while not radio.available(pipe):
                time.sleep(0.0001)
            recv_buffer = []
            radio.read(recv_buffer)
            print(recv_buffer)
            for j in range(0,31):
                temc = chr(recv_buffer[j])  # chr : 아스키 코드 반환
                if temc == 's':
                    smode = 2  #정지 모드
                #if j == 0:
                    #if (temc != '@' and temc != '$'):
                    
                if temc == '!':
                    line = line + ','
                    break;
                
                if temc == '^':
                    smode = 3 #수신 완료
                    plt.close('all')
                    a = plt.axes()
                    print('수신 완료 : ' + str(line))
                    break
                if temc == '*':
                    smode = 4 #YAW 캘리브레이션 모드
                    break;
                else:
                    line = line + temc
                
        
        elif smode == 2:
            print("Stop")
            #tu.SerialSend(2)
            #tu.smode = input()
            smode = 1;
            line = ''
            send_data = '@@'
            ser.write(send_data.encode())
        #if len(temp) != 8:
            #endc = 1
        elif smode == 3:
            temp = line.split(',')
            if temp[0] == '@4':
                #print('n1:' + temp[2][0:5])
                radius[0] = float(temp[2][0:5])
                #print('n2:' + temp[3][1:5])
                radius[1] = float(temp[3][0:5]) 
                #print('n3:' + temp[4][0:5])
                radius[2] = float(temp[4][0:5]) 
                #print('n4:' + temp[5][0:5])
                radius[3] = float(temp[5][0:5]) 
                print('거리 추출 완료 : ' + str(radius[0]) + ' ' + str(radius[1]) + ' ' + str(radius[2]) + ' ' + str(radius[3]))
                line = ''
                smode = 1;
                radius = error_correction(radius)
                print('오차 보정 거리 : ' + str(radius[0]) + ' ' + str(radius[1]) + ' ' + str(radius[2]) + ' ' + str(radius[3]))
                est_position = Final_est_position(r_num, circle_center, radius)
                #circle = []
                for i in range(r_num):
                    #circle.append(a.plot(radius[i]*cyclex + circle_center[i][0], radius[i]*cycley + circle_center[i][1],
                    #             lw=2, color=color[i], label='Anchor'+str(i)))
                    a.plot(radius[i]*cyclex + circle_center[i][0], radius[i]*cycley + circle_center[i][1],
                                 lw=2, color=color[i], label='Anchor'+str(i))
                    plt.scatter(circle_center[i][0], circle_center[i][1], color=color[i], s=15)


                plt.scatter(est_position[0], est_position[1], color='sandybrown', label='est_position', s=30, zorder=6)
                
                #Z_pos = Z_estimation(r_num, circle_center, radius, est_position)
                Z_pos = 1
                est_position.append(Z_pos)
                
                Lidar = float(temp[1])
                MPU = []
                MPU.append(float(temp[6]))
                MPU.append(YAW_cali - float(temp[7]))
                
                correction_lidar = lidar_error_correction(Lidar, MPU)
                target_pos = target_est_pos(est_position, correction_lidar, MPU)
                kkobuk_angle = kkobuk_angle_calc(target_pos, kkobuk_pos)
                
                plt.scatter(target_pos[0], target_pos[1], color='lightgreen', label='target_pos', s=30, zorder=6)
                line_data_X = [target_pos[0], est_position[0]]
                line_data_Y = [target_pos[1], est_position[1]]
                
                plt.plot(line_data_X, line_data_Y, color='deepskyblue', linestyle='--', zorder = 6)
                send_data = '#' + str(kkobuk_angle[0]) + ',' + str(kkobuk_angle[1]) + ',' + str(kkobuk_angle[2]) + ';'  #시리얼 데이터
                print(send_data)
                print(ser.write(send_data.encode()))  #시리얼 데이터 송신
                
                a.set_aspect('equal')
                plt.axis([x_range[0] - 1,x_range[1] + 1, y_range[0] - 1, y_range[1] + 1])
                plt.legend()
                plt.grid(True)
                plt.pause(1)
                plt.savefig('./', dpi=300)
            
            else:
                print('전체 거리가 수신되지 않았습니다!!')
                smode = 1;
                line = ''
        
        elif smode == 4:
            YAW_cali = 0.0
            print('YAW Calibration')
            while 1:
                caliline = ''
                while not radio.available(pipe):
                    time.sleep(0.0001)
                recv_buffer = []
                radio.read(recv_buffer)
                for j in range(0,31):
                    temc = chr(recv_buffer[j])  # chr : 아스키 코드 반환
                    if temc == '^':
                        temp = caliline.split(':')
                        YAW_cali = float(temp[1])
                        print(str(caliline))
                        print(str(YAW_cali))
                        break;
                    if temc == ';':
                        print(str(caliline))
                        break;
                    else:
                        caliline = caliline + temc
                
                if(YAW_cali != 0):
                    print('YAW calibration Done!')
                    smode = 1; #대기모드 
                    break;
            
