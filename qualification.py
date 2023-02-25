import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0
message = ""

# ниже идёт различное HSV для поиска цветов
#[39, 70 ,84] [111, 255, 164] blue
#[ 0, 87,87] [ 66 ,186 ,166] orange

lowblack = np.array([35 ,67 , 0] )
upblack = np.array([103 ,256 , 29])

lowblue = np.array([ 91 ,135,   6])
upblue = np.array([117, 255, 164])

lowo = np.array([  3 , 79 ,113])
upo = np.array([ 59 ,210, 189])

lowr = np.array([ 0, 89, 47])
upr = np.array([  6,223 ,165])

lowg = np.array([ 64, 200 , 56])
upg = np.array([ 81, 255 ,210])


#     d1 = frame[220:260, 0:100]
#     d2 = frame[260:300, 0:200]
#     d3 = frame[220:260, 540:640]
#     d4 = frame[260:300, 440:640]
x=[0, 100, 0, 200, 540, 640, 440, 640]
y=[220,260,260,300,220,260,260,300]

e_old = 0                           #различные переменные для ПД
sr1 = 0
sr2 = 0
speed = 0
perek=0
color_per=None
kp = 0.125
kd = 5
deg=0
u=0
e=0
dat1,dat2=0,0

state=0                             # переменные состояния
stope = 0

t111=time.time()                    # таймеры
tf=time.time()
ts=time.time()

ii = ""                             # для uart

flag_start=False                    # флаги
flag_l=False

vrem_list=[]    # список времени зон для функции x_road()

def black_poisk_l(d1):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  #

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    dat = 0
    for contour in contours:  # этот цикл ищет наибольший контур поплощади и
        # высчитывает среднее значение
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                dat = h * (w+x)
                xm, ym, wm, hm = x, y, w, h
            # кортуры ,но это конфликтовало с поиском кубиков

    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1,dat]

def black_poisk_r(d1,w_dat):
    xm, ym, wm, hm =0,0,0,0
    dat=cv2.GaussianBlur(d1,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(dat.copy(), cv2.COLOR_BGR2HSV)
    blur = cv2.blur(hsv,(5,5))
    mask = cv2.inRange(blur, lowblack, upblack)  #

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #
    max1 = 0
    dat = 0
    for contour in contours:  # этот цикл ищет наибольший контур поплощади и
        # высчитывает среднее значение
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 500:
            if max1 < h * w:
                max1 = h * w
                dat = h * (w_dat-x)
                xm, ym, wm, hm = x, y, w, h
            # кортуры ,но это конфликтовало с поиском кубиков
    return [xm + 1,ym + 1,xm + wm - 1, ym + hm - 1,dat]

def wait_for_key():
    message = str('999999$')
    ii='0'
    while ii == '0':
        port.write(message.encode("utf-8"))
        if port.in_waiting > 0:
            ii = ""
            t = time.time()
            while 1:
                a = str(port.read(), "utf-8")
                if a != '$':
                    ii += a
                else:
                    break
                if t + 0.02 < time.time():
                    break

def pd_regulator(dat1,dat2):              # пропорционально-дифференциальный регулятор
    global e, e_old, deg, color_per, u

    e = dat2 - dat1
    if -5 < e < 5:
        e = 0
    u = int(e * kp + (e - e_old) * kd)
    deg = u
    e_old = e                       # до сюда обычный пропорционально-дифференциальный регулятор

    if dat1 == 0:
        deg = 25
    if dat2 == 0:
        deg = -25

    if dat1 == 0 and dat2 == 0:
        if color_per == "orange":
            deg = -30
        elif color_per == "blue":
            deg = 30

    if deg > 90:
        deg = 90
    if deg < -90:
        deg = -90                   # различные полезные фишки

def bortik_pro():
    global x, y, dat1, dat2

    d1 = frame[y[0]:y[1], x[0]:x[1]]
    d2 = frame[y[2]:y[3], x[2]:x[3]]
    d3 = frame[y[4]:y[5], x[4]:x[5]]
    d4 = frame[y[6]:y[7], x[6]:x[7]]  # забираем часть экрана длядатчиков

    dat1 = (black_poisk_l(d1)[4] + black_poisk_l(d2)[4]) // 100  # высчитываем среднее исходя из показаний датчиков
    dat2 = (black_poisk_r(d3, 125)[4] + black_poisk_r(d4, 250)[4]) // 100





wait_for_key()

while 1:
    key = robot.get_key()
    frame = robot.get_frame(wait_new_frame=1)

    l1 = black_poisk_l(frame[y[0]:y[1], x[0]:x[1]])
    cv2.rectangle(frame[y[0]:y[1], x[0]:x[1]],(l1[0],l1[1]),(l1[2],l1[3]), (10, 245, 0), 2)
    cv2.rectangle(frame, (x[0], y[0]), (x[1], y[1]), (0, 25,200), 2)

    l2 = black_poisk_l(frame[y[2]:y[3], x[2]:x[3]])
    cv2.rectangle(frame[y[2]:y[3], x[2]:x[3]], (l2[0], l2[1]), (l2[2], l2[3]), (10, 245, 0), 2)
    cv2.rectangle(frame, (x[2], y[2]), (x[3], y[3]), (0, 25, 200), 2)

    l3 = black_poisk_r(frame[y[4]:y[5], x[4]:x[5]], 100)
    cv2.rectangle(frame[y[4]:y[5], x[4]:x[5]], (l3[0], l3[1]), (l3[2], l3[3]), (10, 245, 0), 2)
    cv2.rectangle(frame, (x[4], y[4]), (x[5], y[5]), (0, 25, 200), 2)

    l4 = black_poisk_r(frame[y[6]:y[7], x[6]:x[7]], 200)
    cv2.rectangle(frame[y[6]:y[7], x[6]:x[7]], (l4[0], l4[1]), (l4[2], l4[3]), (10, 245, 0), 2)
    cv2.rectangle(frame, (x[6], y[6]), (x[7], y[7]), (0, 25, 200), 2)

    pd_regulator(dat1, dat2)

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    message = str(int(deg) + 200) + str(int(speed) + 200) + '$'
    port.write(message.encode("utf-8"))

    robot.text_to_frame(frame, 'fps = ' + str(fps), 50, 20)
    robot.text_to_frame(frame, l1[3], 0, 140)
    robot.text_to_frame(frame, l2[3], 600, 140)
    robot.text_to_frame(frame, u, 300, 200)
    robot.set_frame(frame, 40)
