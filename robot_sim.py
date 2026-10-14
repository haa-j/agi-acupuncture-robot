import pybullet as p
import pybullet_data
import time

class AcupunctureRobotSim:
    def __init__(self):
        # 连接仿真窗口，GUI可视化
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # 加载地面
        self.planeId = p.loadURDF("plane.urdf")
        # 加载机械臂，使用pybullet自带kuka iiwa 7轴机械臂
        self.robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

        # 获取机械臂关节信息
        self.numJoints = p.getNumJoints(self.robotId)
        self.endEffectorIndex = 6  # 末端执行器关节编号
        self.jointIndices = list(range(self.numJoints))

        # 初始关节角度
        self.startJoints = [0.0] * self.numJoints
        for i in range(self.numJoints):
            p.resetJointState(self.robotId, i, self.startJoints[i])

    def move_to_target(self, target_pos, target_orn=p.getQuaternionFromEuler([0, 3.14, 0])):
        """
        移动机械臂末端到目标穴位坐标
        :param target_pos: [x,y,z] 三维坐标
        :param target_orn: 末端姿态
        """
        # 逆运动学求解关节角度
        jointPoses = p.calculateInverseKinematics(
            self.robotId,
            self.endEffectorIndex,
            target_pos,
            target_orn,
            maxNumIterations=100,
            residualThreshold=1e-3
        )
        # 控制关节运动
        for i in range(self.numJoints):
            p.setJointMotorControl2(
                bodyIndex=self.robotId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=jointPoses[i],
                force=500
            )
        # 仿真步进，等待运动完成
        for _ in range(120):
            p.stepSimulation()
            time.sleep(1./240.)

    def simulate_acupuncture(self, hole_z_offset=0.02):
        """模拟针刺动作：向下扎入，再抬起"""
        # 获取当前末端位置
        current_pos, current_orn = p.getLinkState(self.robotId, self.endEffectorIndex)[0:2]
        # 下针
        down_pos = [current_pos[0], current_pos[1], current_pos[2] - hole_z_offset]
        self.move_to_target(down_pos, current_orn)
        time.sleep(0.5)
        # 抬针
        self.move_to_target(current_pos, current_orn)

    def close(self):
        p.disconnect()

if __name__ == "__main__":
    # 单独测试仿真模块
    sim = AcupunctureRobotSim()
    try:
        # 测试穴位目标点，你后续替换成AGI输出转换后的坐标
        acupoint_target = [0.4, 0.2, 0.3]
        print(f"移动到穴位坐标：{acupoint_target}")
        sim.move_to_target(acupoint_target)
        time.sleep(0.8)
        sim.simulate_acupuncture()
        time.sleep(3)
    finally:
        sim.close()
