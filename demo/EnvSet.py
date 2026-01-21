'''
# 创建障碍物（柱子，创建，一定范围内随机）
# 控制小车行走，轨迹随机

H:810
Car:814

'''




import UE4CtrlAPI
import math
import time


ue = UE4CtrlAPI.UE4CtrlAPI()

ue.sendUE4Cmd(cmd = "RflyChangeMapbyName MatchScene2025")

#ue.sendUE4PosScale(1,10100310)
#time.sleep(2)
ue.sendUE4ExtAct(1,[1, -35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

#设置起飞标志
ue.sendUE4PosScale(copterID=100001,vehicleType=810,PosE=[0,0,0])

#设置静态柱子
ue.sendUE4PosScale(copterID=100002,vehicleType=824,PosE=[3.6,0,0])
ue.sendUE4PosScale(copterID=100003,vehicleType=824,PosE=[6,0,0])

ue.sendUE4PosScale(copterID=100004,vehicleType=824,PosE=[5,1.4,0])
ue.sendUE4PosScale(copterID=100005,vehicleType=824,PosE=[5,1.4,0])
ue.sendUE4PosScale(copterID=100006,vehicleType=824,PosE=[5,-1.4,0])
ue.sendUE4PosScale(copterID=100007,vehicleType=824,PosE=[7,-1.4,0])
ue.sendUE4PosScale(copterID=100008,vehicleType=824,PosE=[7,1.4,0])


ue.sendUE4PosScale(copterID=100009,vehicleType=820,PosE=[3.34,1.85,0])
ue.sendUE4PosScale(copterID=100010,vehicleType=820,PosE=[3.34,-1.85,0])
ue.sendUE4Pos(1010,100000823,0,[2.1,0,0],[0,0,0])
ue.sendUE4Pos(1011,100000822,0,[7.17,0,0],[0,0,0])

# ue.sendUE4ExtAct
# ue.sendUE4Pos(copterID=1000009,vehicleType=820,PosE=[7.5,0,0]) 

#以下代码调试使用Python 控小车

# ue.sendUE4PosScale(copterID=2,vehicleType=825,PosE=[7.17,0,0])
ue.sendUE4PosScale(copterID=2,vehicleType=825,PosE=[5,-1,0])
time.sleep(5)
ue.sendUE4ExtAct(2,[1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])



path1 = [[7.17,0],[6.31,-0.72],[3.24,-0.72],[2.15,0],[3.24,-0.72],[6.31,-0.72],[7.17,0]]
path2 = [[7.17,0],[6.31,0.75],[3.24,0.85],[2.15,0],[3.24,0.85],[6.31,0.75],[7.17,0]]
path3 = [[7.17,0],[6.31,-0.72],[3.24,-0.72],[2.15,0],[3.24,0.72],[6.31,0.86],[7.17,0]]
path4 = [[7.17,0],[6.31,-0.72],[5.52,-0.72],[4.57,0.49],[3.24,0.72],[2.15,0],[3.24,0.72],[4.57,0.72],[5.58,-0.72],[6.31,-0.72],[7.17,0]]
path5 = [[7.17,0],[6.31,0.72],[5.52,0.72],[4.57,-0.49],[3.24,-0.72],[2.15,0],[3.24,-0.72],[4.57,-0.72],[5.58,0.72],[6.31,0.72],[7.17,0]]
path6 =[[7.17,0],[6.31,-0.72],[5.52,-0.72],[4.57,0.49],[3.24,0.72],[2.15,0],[3.24,-0.72],[4.57,-0.72],[5.58,0.72],[6.31,0.72],[7.17,0]] 

offset = 0.03
path = path1
idx = 0
yaw = 0
pos=[7.17,0,0]
offset_yaw = 0.01
tgt_yaw = 0
while True:

    if(idx == len(path)-1):
        idx = 0
    if(abs(pos[0]- path[idx][0]) < offset and abs(pos[1]-path[idx][1])< offset):
        dx = path[idx+1][0] - path[idx][0]
        dy = path[idx+1][1] - path[idx][1]
        pos[0] = path[idx][0]
        pos[1] = path[idx][1]
        idx += 1
        yaw = math.atan2(dy,dx)
    offset_x =  math.cos(yaw)*offset
    offset_y = math.sin(yaw) *offset

    pos[0] += offset_x
    pos[1] += offset_y
    

    ue.sendUE4PosScale(copterID=2,vehicleType=825,PosE=pos,AngEuler=[0,0,yaw])
    time.sleep(0.05)