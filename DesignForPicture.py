"""
利用深度图像进行躲避障碍的程序
"""
import airsim
import numpy as np
import math
import time
import os
# import random

Point = airsim.YawMode()
Point.is_rate = False
Point.yaw_or_rate = 0


"""
参数初始化
"""
Height = -7
Speed = 3
AvoidSpeed = 4
FlyTime = 10000
CollisionLimit = 6  # 无人机飞行碰撞体积参数，是一个比机身的边长大一些的值
DistanceCollector = np.array([0, 0, 0], dtype=np.float16)  # 用于储存无人机飞行方向的传感器数量
DistanceLimit = np.array([CollisionLimit / 2 * math.sqrt(2) + 0, CollisionLimit,
                          CollisionLimit / 2 * math.sqrt(2) + 0], dtype=np.float16)
PictureCollector = np.array([0, 0, 0, 0])  # 将图片分为四个部分
# WaitingTime = 35  # 等待无人机停稳的时间
CollisionCount = 0  # 记录冲突的次数
ConditionWait = "Wait For Plane to Be Still!"
ConditionAnalysis = "Now Avoiding Obstacle!"
ConditionFly = "Now No Collision!"
RandomNumber = 0
WhitePoint = 10
FinalPoint = np.array([0, 0], dtype=np.float16)  # 用于储存终点的坐标（x，y）
CoolTime = 4  # 给无人机转向的时间

"""
无人机飞行的准备阶段
"""
# 与无人机建立联系并且创建记录文档
fp = open('F:/RecordingForCamera'+str(CollisionLimit) + '.txt', 'a+')  # 新建文件夹
client = airsim.MultirotorClient()
client.confirmConnection()

# 取得无人机的控制
client.enableApiControl(True)

# 解锁并且附加移动轨迹
client.armDisarm(True)
client.simSetTraceLine([0, 0, 0, 1], 20.0, "Drone1")  # 规定无人机轨迹的三基色以及透明度
client.moveToZAsync(Height, 2).join()  # 上升到5米的高度
x = client.simGetVehiclePose("Drone1").position.x_val
y = client.simGetVehiclePose("Drone1").position.y_val  # 获取无人机所在的位置
FinalPoint[0] = client.simGetObjectPose("FinalMark").position.x_val
FinalPoint[1] = client.simGetObjectPose("FinalMark").position.y_val
# print(client.simGetObjectPose("FinalMark").position)

# 代码检测
print("Position of Plane:", x, y)
print("FinalPoint: ", FinalPoint[0], FinalPoint[1])
Distance = math.sqrt((FinalPoint[1] - y)**2 + (FinalPoint[0] - x)**2)  # 用于记录无人机与重点之间的距离
Angle = (math.atan2((FinalPoint[1] - y), (FinalPoint[0] - x))) / 2 / math.pi * 360  # 获得起始的飞行角度,单位是°
print('Experiment Start at', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), "Direction:",
      Angle, "Distance remain:", Distance, "cm", file=fp)
print('Speed:', Speed, 'Limitation:', CollisionLimit, file=fp)
client.moveByVelocityZAsync(Speed * math.cos(Angle / 360 * 2 * math.pi), Speed * math.sin(Angle / 360 * 2 * math.pi),
                            Height, FlyTime, drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=Point, vehicle_name="Drone1")

while Distance > 10:
    DistanceCollector[0] = client.getDistanceSensorData("Distance14").distance
    DistanceCollector[1] = client.getDistanceSensorData("Distance0").distance
    DistanceCollector[2] = client.getDistanceSensorData("Distance2").distance
    x = client.simGetVehiclePose("Drone1").position.x_val
    y = client.simGetVehiclePose("Drone1").position.y_val
    Distance = math.sqrt((FinalPoint[1] - y)**2 + (FinalPoint[0] - x)**2)  # 用于记录无人机到终点的距离
    if Distance < 10:
        break

    if (DistanceCollector[0] <= DistanceLimit[0] or DistanceCollector[1] <= DistanceLimit[1]
            or DistanceCollector[2] <= DistanceLimit[2]):
        # print("Begin to stop!")
        client.simPrintLogMessage("Current Condition: ", ConditionWait, 3)
        x = client.simGetVehiclePose("Drone1").position.x_val
        y = client.simGetVehiclePose("Drone1").position.y_val
        client.moveToPositionAsync(x-1, y-1, Height, 1)
        Distance = math.sqrt((FinalPoint[1] - y)**2 + (FinalPoint[0] - x)**2)  # 用于记录无人机与重点之间的距离
        # print('Distance:', Distance)
        # print('Position is', x, y)

        # client.hoverAsync()
        # time.sleep(40)
        client.simPrintLogMessage("Current Condition: ", ConditionAnalysis, 3)
        responses = client.simGetImages([airsim.ImageRequest
                                         ("Camera0", airsim.ImageType.DepthPlanner, True, False)])
        response = responses[0]
        # 将Python列表转换为二维数组
        img_rgb = airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
        # print(img_rgb[10, 10])

        # 统计图像中的白点
        for i in range(0, 256):
            for j in range(36, 108):
                if img_rgb[j, i] > WhitePoint and i <= 63:
                    PictureCollector[0] = PictureCollector[0] + 1
                elif img_rgb[j, i] > WhitePoint and 63 < i <= 127:
                    PictureCollector[1] = PictureCollector[1] + 1
                elif img_rgb[j, i] > WhitePoint and 127 < i <= 191:
                    PictureCollector[2] = PictureCollector[2] + 1
                elif img_rgb[j, i] > WhitePoint and 191 < i <= 255:
                    PictureCollector[3] = PictureCollector[3] + 1

        # 打印收集到的最终结果
        # print(PictureCollector[0])
        # print(PictureCollector[1])
        # print(PictureCollector[2])
        # print(PictureCollector[3])

        # 根据像素中的白点决定接下来的方向
        # 前方无路则返回，但具有随机性
        # RandomNumber = random.randint(-30, 30)
        if (PictureCollector[0] <= 100 and PictureCollector[1] <= 100
                and PictureCollector[2] <= 100 and PictureCollector[3] <= 100):
            Angle = Angle - 90
        else:
            if np.argmax(PictureCollector) == 0:
                Angle = Angle - 45 * 2
                CoolTime = 4
            elif np.argmax(PictureCollector) == 1:
                Angle = Angle - 45
                CoolTime = 4
            elif np.argmax(PictureCollector) == 2:
                Angle = Angle + 45
                CoolTime = 4
            elif np.argmax(PictureCollector) == 3:
                Angle = Angle + 45 * 2
                CoolTime = 4

        # 更改无人机的运行方向并在文本中记录数据
        # client.simPrintLogMessage("Current Condition: ", ConditionFly, 3)
        client.moveByVelocityZAsync(AvoidSpeed * math.cos(Angle / 360 * 2 * math.pi),
                                    AvoidSpeed * math.sin(Angle / 360 * 2 * math.pi),
                                    Height, FlyTime, drivetrain=airsim.DrivetrainType.ForwardOnly,
                                    yaw_mode=Point, vehicle_name="Drone1")
        print(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), "No.", str(CollisionCount), 'Angle:',
              Angle, "Position:", x, '**', y, file=fp)
        # time.sleep(3)

        # 将像素记录数组置零
        PictureCollector[0] = 0
        PictureCollector[1] = 0
        PictureCollector[2] = 0
        PictureCollector[3] = 0

        # 保存用来处理冲突的图像
        # airsim.write_png(os.path.normpath("F:/CollisionPicture" + str(CollisionCount) + '.png'), img_rgb)

        CollisionCount = CollisionCount + 1
        time.sleep(3)

    else:
        x = client.simGetVehiclePose("Drone1").position.x_val
        y = client.simGetVehiclePose("Drone1").position.y_val
        Angle = (math.atan2((FinalPoint[1] - y), (FinalPoint[0] - x))) / 2 / math.pi * 360  # 获得起始的飞行角度,单位是°
        # print(Angle)
        client.moveByVelocityZAsync(Speed * math.cos(Angle / 360 * 2 * math.pi),
                                    Speed * math.sin(Angle / 360 * 2 * math.pi),
                                    Height, FlyTime, drivetrain=airsim.DrivetrainType.ForwardOnly,
                                    yaw_mode=Point, vehicle_name="Drone1")
        Distance = math.sqrt((FinalPoint[1] - y) ** 2 + (FinalPoint[0] - x) ** 2)  # 用于记录无人机与重点之间的距离
        client.simPrintLogMessage("Current Condition: ", ConditionFly, 3)
        # print(Distance)
        # print(client.simGetObjectPose("FinalMark"))
        # print(x, y)
        time.sleep(1)
        # print(Distance)


"""
无人机测试完成，降落，接触控制并且上锁
"""
# 悬停3秒钟
print('Experiment Ends at', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), file=fp)
print('********************************************************************************************', file=fp)
client.hoverAsync().join()
time.sleep(3)
client.landAsync().join()
client.reset()
# 解锁
client.armDisarm(False)

# 解开API控制
client.enableApiControl(False)
fp.close()
