"""
八个测距仪工作时候的代码
"""
import airsim
import time
import math
import numpy as np

Point = airsim.YawMode()
Point.is_rate = False
Point.yaw_or_rate = 0

"""
用于对各个传感器数据进行处理并得出最终躲障方向的方程(八个传感器的版本)
"""


def direction_set(array_of_distance_sensor, clock):
    array_temp = np.array([0, 0, 0, 0, 0], dtype=np.float16)
    position = 0  # 用来遍历数组的索引参数

    while position < 5:
        if position == 0 or position == 4:
            array_temp[position] = array_of_distance_sensor[(clock - 2 + position) % 8] / 20 * (
                    0.5 * array_of_distance_sensor[(clock - 3 + position) % 8] +
                    array_of_distance_sensor[(clock - 2 + position) % 8] +
                    0.5 * array_of_distance_sensor[(clock - 1 + position) % 8])
        else:
            array_temp[position] = array_of_distance_sensor[(clock - 2 + position) % 8] / 20 * (
                0.5 * array_of_distance_sensor[(clock - 3 + position) % 8] +
                array_of_distance_sensor[(clock - 2 + position) % 8] +
                0.5 * array_of_distance_sensor[(clock - 1 + position) % 8])

        position += 1

    return (clock - 2 + np.argmax(array_temp)) % 8  # 返回用于决策的飞行方向


"""
根据航向设置各个方向的阈值的方程(8个测距仪的版本)
"""


def limit_set(direction):
    if direction % 2 == 0:
        # 设置飞行方向以及左右四个传感器的判决距离
        NearRound[direction % 8] = CollisionLimit + 0
        NearRound[(direction + 1) % 8] = CollisionLimit / 2 * math.sqrt(2)
        NearRound[(direction + 2) % 8] = CollisionLimit / 2
        NearRound[(direction - 1) % 8] = CollisionLimit / 2 * math.sqrt(2)
        NearRound[(direction - 2) % 8] = CollisionLimit / 2
        # 飞行方向之后的三个传感器的判决距离全部置零
        NearRound[(direction + 3) % 8] = 0
        NearRound[(direction - 3) % 8] = 0
        NearRound[(direction + 4) % 8] = 0

    else:
        # 设置飞行方向以及左右四个传感器的判决距离
        NearRound[direction % 8] = CollisionLimit * math.sqrt(2) + 0
        NearRound[(direction + 1) % 8] = CollisionLimit
        NearRound[(direction + 2) % 8] = CollisionLimit / 2 * math.sqrt(2)
        NearRound[(direction - 1) % 8] = CollisionLimit
        NearRound[(direction - 2) % 8] = CollisionLimit / 2 * math.sqrt(2)
        # 飞行方向之后的三个传感器的判决距离全部置零
        NearRound[(direction + 3) % 8] = 0
        NearRound[(direction - 3) % 8] = 0
        NearRound[(direction + 4) % 8] = 0


"""
初始化参数
"""
DistanceCollector = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float16)  # 用于记录冲突时各个传感器的数据
NearRound = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float16)  # 用于记录中间区域判决的阈值
FinalPoint = np.array([0, 0], dtype=np.float16)  # 记录终点的坐标
AngleStep = 45  # 指针的步长
FlyTime = 10000  # 无人机按照指定方向默认飞行的时间
latitude = 0  # 无人机与下方物体的距离(相对高度)
CollisionLimit = 6  # 无人机飞行碰撞体积参数，是一个比机身的边长大一些的值
ChangeCalm = 0  # 用于冷却躲避障碍的动作和冷却文件写入信息的频率。0表示冷却完成，非零表示正在冷却
VerticalChange = 0  # 用于防止无人机进入死循环
printCD = 0  # 初始化打印CD
SignOfNormal = "No Possible Collision, Hold Speed!"
SignOfChange = "Find Collision! Now Reorient!"
Speed = 3  # 无人机飞行速度
AvoidSpeed = 4
AvoidTime = 2.5
Clock = 0  # 方向指针
limit_set(Clock)
Height = -7  # 无人机飞行保持的高度
CollisionCount = 0  # 记录冲突避障的字数
# LastAngle = 0  # 用于记录冲突的方向

"""
无人机飞行的准备阶段
"""
# 与无人机建立联系并且创建记录文档
fp = open('F:/RecordingForAvoidTime' + str(CollisionLimit) + str(Speed) + '.txt', 'a+')  # 新建文件夹
client = airsim.MultirotorClient()
client.confirmConnection()
# client.simPrintLogMessage("Avoidance Direction Now: ", str(Clock * AngleStep))

# 取得无人机的控制
client.enableApiControl(True)

# 解锁并且附加移动轨迹
client.armDisarm(True)
client.simSetTraceLine([0, 0, 0, 1], 20.0, "Drone1")  # 规定无人机轨迹的三基色以及透明度

# client.takeoffAsync().join()  # 无人机起飞到2米的高度
client.moveToZAsync(Height, 2).join()  # 上升到5米的高度
x = client.simGetVehiclePose("Drone1").position.x_val
y = client.simGetVehiclePose("Drone1").position.y_val  # 获取无人机所在的位置
FinalPoint[0] = client.simGetObjectPose("FinalMark").position.x_val
FinalPoint[1] = client.simGetObjectPose("FinalMark").position.y_val  # 获得重点的位置
SpeedAngLe = math.atan2(FinalPoint[1]-y, FinalPoint[0]-x)
Distance = math.sqrt((FinalPoint[1]-y)**2 + (FinalPoint[0])**2)
print('Experiment Start at', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), "Direction:",
      SpeedAngLe, "Distance:", Distance, file=fp)
print('Speed:', Speed, 'Limitation:', (CollisionLimit + 0), 'AvoidTime:', AvoidTime, file=fp)
client.moveByVelocityZAsync(Speed * math.cos(SpeedAngLe), Speed * math.sin(SpeedAngLe), Height,
                            FlyTime)  # 无人机的初始飞行指令

"""
无人机避障实验阶段，完成20次冲突解决次数,在冲突时在指定文档中写入一些相关数据并且在无人机窗口打印
"""
while Distance > 10:
    client.simPrintLogMessage("Current Condition:", SignOfNormal, 3)
    # client.simPrintLogMessage("Height: ", str(client.getDistanceSensorData("Distance16").distance))
    # latitude = client.getDistanceSensorData("Distance16").distance

    # 八个测距仪时候的距离存储
    DistanceCollector[0] = client.getDistanceSensorData("Distance0").distance
    DistanceCollector[1] = client.getDistanceSensorData("Distance2").distance
    DistanceCollector[2] = client.getDistanceSensorData("Distance4").distance
    DistanceCollector[3] = client.getDistanceSensorData("Distance6").distance
    DistanceCollector[4] = client.getDistanceSensorData("Distance8").distance
    DistanceCollector[5] = client.getDistanceSensorData("Distance10").distance
    DistanceCollector[6] = client.getDistanceSensorData("Distance12").distance
    DistanceCollector[7] = client.getDistanceSensorData("Distance14").distance

    # 只检测无人机运动方向上共5个距离传感器的距离,如果遇到障碍就停下并且上一个动作不是躲避障碍，改变航向后再继续飞行
    if (((DistanceCollector[Clock % 8] <= NearRound[Clock]) or
         (DistanceCollector[(Clock + 1) % 8] <= NearRound[((Clock + 1) % 8)]) or
         (DistanceCollector[(Clock - 1) % 8] <= NearRound[((Clock - 1) % 8)]))):

        # 修改航向具有最高优先级，得出结果并立即执行，而且更改测距阈值
        # 根据数组的内容获得规避的方向并且记录在文件中
        # time.sleep(0.5)
        DistanceCollector[0] = client.getDistanceSensorData("Distance0").distance
        DistanceCollector[1] = client.getDistanceSensorData("Distance2").distance
        DistanceCollector[2] = client.getDistanceSensorData("Distance4").distance
        DistanceCollector[3] = client.getDistanceSensorData("Distance6").distance
        DistanceCollector[4] = client.getDistanceSensorData("Distance8").distance
        DistanceCollector[5] = client.getDistanceSensorData("Distance10").distance
        DistanceCollector[6] = client.getDistanceSensorData("Distance12").distance
        DistanceCollector[7] = client.getDistanceSensorData("Distance14").distance
        Clock = direction_set(DistanceCollector, Clock)
        SpeedAngLe = Clock * AngleStep / 360 * 2 * math.pi + SpeedAngLe
        client.moveByVelocityZAsync(AvoidSpeed * math.cos(SpeedAngLe), AvoidSpeed * math.sin(SpeedAngLe), Height,
                                    FlyTime)  # 修改无人机的方向
        # limit_set(Clock)  # 根据新的方向设置新的阈值
        # LastAngle = np.argmin(DistanceCollector)
        # ChangeCalm = 5  # 避免多次记录同一次冲突
        x = client.simGetVehiclePose("Drone1").position.x_val
        y = client.simGetVehiclePose("Drone1").position.y_val  # 获取无人机所在的位置
        Distance = math.sqrt((FinalPoint[1] - y) ** 2 + (FinalPoint[0] - x) ** 2)  # 刷新距离
        print(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), "No." + str(CollisionCount),
              "AvoidanceDirection:", Clock * AngleStep,
              "CollisionDirection: ", np.argmin(DistanceCollector) * AngleStep,
              "Position:", x, ',,', y, file=fp)
        CollisionCount += 1  # 记录数加一
        # print(CollisionCount)
        # 在界面上输出当前无人机的飞行方向
        client.simPrintLogMessage("Avoidance Direction Now: ",
                                  str(SpeedAngLe))
        client.simPrintLogMessage("Current Condition:", SignOfChange, 3)
        # x = client.simGetVehiclePose("Drone1").position.x_val
        # y = client.simGetVehiclePose("Drone1").position.y_val  # 获取无人机所在的位置
        # Distance = math.sqrt((FinalPoint[1] - y) ** 2 + (FinalPoint[0]) ** 2)  # 刷新距离
        time.sleep(AvoidTime)

    else:
        x = client.simGetVehiclePose("Drone1").position.x_val
        y = client.simGetVehiclePose("Drone1").position.y_val  # 获取无人机所在的位置
        Clock = 0
        limit_set(Clock)
        SpeedAngLe = math.atan2(FinalPoint[1] - y, FinalPoint[0] - x)
        client.moveByVelocityZAsync(Speed * math.cos(SpeedAngLe),
                                    Speed * math.sin(SpeedAngLe),
                                    Height, FlyTime, drivetrain=airsim.DrivetrainType.ForwardOnly,
                                    yaw_mode=Point, vehicle_name="Drone1")
        Distance = math.sqrt((FinalPoint[1] - y) ** 2 + (FinalPoint[0] - x) ** 2)


"""
无人机测试完成，降落，接触控制并且上锁
"""
# 悬停6秒钟
print('Experiment Ends at', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), file=fp)
print('********************************************************************************************', file=fp)
client.hoverAsync().join()
time.sleep(6)
client.landAsync().join()
time.sleep(2)
# 将无人机放回原位
client.reset()
# 解锁
client.armDisarm(False)

# 解开API控制
client.enableApiControl(False)
fp.close()
