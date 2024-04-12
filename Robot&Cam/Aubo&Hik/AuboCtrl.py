from AuboClass import robotcontrol as RC
import math


class AuboCtrl:
    
    def __init__(self):
        RC.Auboi5Robot.initialize()
        self.robot = RC.Auboi5Robot()
        pass


    def ConnectToAubo(self, ip='localhost',port=8899):
        '''
        * Function:     ConnectToAubo
        * Description:  连接至Aubo协作臂
        * Inputs:       ip: 协作臂的ip地址
                        port: 协作臂的端口号
        * Outputs:      
        * Returns:      连接结果
                            0: 连接失败
                            1: 连接成功
        * Notes:
        '''
        RC.logger_init()
        RC.logger.info("{0} connecting...".format(RC.Auboi5Robot.get_local_time()))
        handle = self.robot.create_context()
        RC.logger.info("robot.rsh={0}".format(handle))
        
        queue = RC.Queue()

        p = RC.Process(target=RC.runWaypoint, args=(queue,))
        p.start()
        RC.time.sleep(5)
        print("process started.")

        
        result = self.robot.connect(ip , port)

        if result != RC.RobotErrorType.RobotError_SUCC:
            RC.logger.info("connect server{0}:{1} failed.".format(ip, port))
            return 0
        else:
            RC.logger.info("connect server{0}:{1} successed.".format(ip, port))
            return 1

    def SetVelocity(self,joint_velocity,joint_acceleration):
        '''
        * Function:     SetVelocity
        * Description:  设置Aubo协作臂的速度
        * Inputs:       joint_velocity: 关节速度 
                        joint_acceleration: 关节加速度
        * Outputs:      
        * Returns:      
        * Notes:
        '''

        joint_maxvelc = (joint_velocity*0.834736, joint_velocity*0.834736, joint_velocity*0.834736, joint_velocity, joint_velocity, joint_velocity)
    
        joint_maxacc = (joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration, joint_acceleration)
    
        self.robot.set_joint_maxacc(joint_maxacc)
        self.robot.set_joint_maxvelc(joint_maxvelc)
        self.robot.set_arrival_ahead_blend(0.05)

    def MoveAubo(self,target_pose,target_rpy, joint_velocity,joint_acceleration):
        '''
        * Function:     MoveAubo
        * Description:  控制Aubo协作臂点动
        * Inputs:       target_pose: 目标点位的笛卡尔坐标
                        target_rpy: 目标点位的偏转角
                        joint_velocity: 关节速度 
                        joint_acceleration: 关节加速度
        * Outputs:      
        * Returns:      1: 到位完成
        * Notes:
        '''
        self.robot.enable_robot_event()
        self.robot.init_profile()
        self.SetVelocity(joint_velocity,joint_acceleration)
        self.robot.move_to_target_in_cartesian(target_pose,target_rpy)

        # 当法兰的实际位置与目标位置的距离小于0.5mm时，才跳出循环进入下一阶段
        while(1):
            current_data = self.robot.get_current_waypoint()
            current_pos = current_data['pos']
            dist = math.sqrt((current_pos[0] - target_pose[0])**2 + (current_pos[1] - target_pose[1])**2 + (current_pos[2] - target_pose[2])**2)
            if dist <= 0.0005:
                return 1
            
    