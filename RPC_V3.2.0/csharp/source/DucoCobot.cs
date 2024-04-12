using System;
using System.Collections.Generic;
using Thrift.Transport;
using Thrift.Protocol;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Numerics;

namespace DucoRPC
{
    /**
	 * @brief 机器人状态信息
	 */
    enum StateRobot
    {
        SR_Start = 0,
        SR_Initialize = 1,
        SR_Logout = 2,
        SR_Login = 3,
        SR_PowerOff = 4,
        SR_Disable = 5,
        SR_Enable = 6
    };

    /**
	 * @brief 程序状态信息
	 */
    enum StateProgram
    {
        SP_Stopped = 0,
        SP_Stopping = 1,
        SP_Running = 2,
        SP_Paused = 3,
        SP_Pausing = 4
    };


    /**
	 * @brief 机器人操作模式信息
	 */
    enum OperationMode
    {
        kManual = 0,
        kAuto = 1,
        kRemote = 2
    };

    /**
	 * @brief 任务状态信息
	 */
    enum TaskState
    {
        ST_Idle = 0,
        ST_Running = 1,
        ST_Paused = 2,
        ST_Stopped = 3,
        ST_Finished = 4,
        ST_Interrupt = 5,
        ST_Error = 6,
        ST_Illegal = 7,
        ST_ParameterMismatch = 8
    };

    /**
	 * @brief 安全控制器状态信息
	 */
    enum SafetyState
    {
        SS_INIT = 0,
        SS_WAIT = 2,
        SS_CONFIG = 3,
        SS_POWER_OFF = 4,
        SS_RUN = 5,
        SS_RECOVERY = 6,
        SS_STOP2 = 7,
        SS_STOP1 = 8,
        SS_STOP0 = 9,
        SS_MODEL = 10,
        SS_REDUCE = 12,
        SS_BOOT = 13,
        SS_FAIL = 14,
        SS_UPDATE = 99
    };
    /**
     * 机器人移动过程中对控制柜的IO操作数据
     */
    public struct OP
    {
        public sbyte time_or_dist_1;       // 轨迹起始点触发类型, 0:不启用, 1:时间触发, 2:距离触发
        public sbyte trig_io_1;            // 轨迹触发控制柜IO的输出序号, 范围1-16
        public bool trig_value_1;          // 轨迹触发控制柜IO的电平高低, false:低电平, true:高电平
        public double trig_time_1;         // 当time_or_dist_1为1时, 代表轨迹运行多少时间长度触发IO,单位: ms
        public double trig_dist_1;         // 当time_or_dist_1为2时, 代表轨迹运行多少距离长度触发IO,单位: m
        public string trig_event_1;        // 轨迹触发的用户自定义事件名称
        public sbyte time_or_dist_2;       // 轨迹结束点触发类型, 0:不启用, 1:时间触发, 2:距离触发
        public sbyte trig_io_2;            // 轨迹触发控制柜IO的输出序号, 范围1-16
        public bool trig_value_2;          // 轨迹触发控制柜IO的电平高低, false:低电平, true:高电平
        public double trig_time_2;         // 当time_or_dist_2为1时, 当trig_time_2 >= 0时, 代表轨迹运行剩余多少时间长度触发IO,单位: ms; 当trig_time_2 < 0时, 代表代表轨迹运行结束后多少时间长度触发IO
        public double trig_dist_2;         // 当time_or_dist_2为2时, 当trig_ dist _2 >= 0时, 代表轨迹运行剩余多少距离长度触发IO,单位: m;当trig_ dist _2 < 0时, 代表代表轨迹运行结束后多少距离长度触发IO
        public string trig_event_2;        // 轨迹触发的用户自定义事件名称

        public OP(sbyte v1, sbyte v2, bool v3, double v4, double v5, string v6, sbyte v7, sbyte v8, bool v9, double v10, double v11, string v12) : this()
        {
            this.time_or_dist_1 = v1;
            this.trig_io_1 = v2;
            this.trig_value_1 = v3;
            this.trig_time_1 = v4;
            this.trig_dist_1 = v5;
            this.trig_event_1 = v6;
            this.time_or_dist_2 = v7;
            this.trig_io_2 = v8;
            this.trig_value_2 = v9;
            this.trig_time_2 = v10;
            this.trig_dist_2 = v11;
            this.trig_event_2 = v12;
        }
    }
    /*
   // 机器人相关信息
   public struct RobotStatus
   {
       List<double> jointExpectPosition;   // 目标关节位置
       List<double> jointExpectVelocity;   // 目标角速度
       List<double> jointExpectAccelera;   // 目标角加速度
       List<double> jointActualPosition;   // 实际关节位置
       List<double> jointActualVelocity;   // 实际角速度
       List<double> jointActualAccelera;   // 实际角加速度
       List<double> jointActualCurrent;    // 实际关节电流
       List<double> jointTemperature;      // 时间关节温度
       List<double> driverTemperature;     // 未使用
       List<double> cartExpectPosition;    // 目标末端位姿
       List<double> cartExpectVelocity;    // 目标末端速度
       List<double> cartExpectAccelera;    // 目标末端加速度
       List<double> cartActualPosition;    // 实际末端位姿
       List<double> cartActualVelocity;    // 实际末端速度
       List<double> cartActualAccelera;    // 实际末端加速度
       List<bool> slaveReady;            // 从站状态
       bool collision;       // 是否发生碰撞
       sbyte collisionAxis;   // 发生碰撞的关节
       bool emcStopSignal;   // 未使用
       sbyte robotState;      // 机器人状态
       int robotError;      // 机器人错误码
   };

   // IO和寄存器相关信息
   public struct IOStatus
   {
       List<double> analogCurrentOutputs;  // 模拟电流输出
       List<double> analogVoltageOutputs;  // 模拟电压输出
       List<double> analogCurrentInputs;   // 模拟电流输入
       List<double> analogVoltageInputs;   // 模拟电压输入
       List<bool> digitalInputs;         // 通用数字输入
       List<bool> digitalOutputs;        // 通用数字输出
       List<bool> toolIOIn;              // 工具数字输入
       List<bool> toolIOOut;             // 工具数字输出
       List<bool> toolButton;            // 工具末端按键
       List<bool> funRegisterInputs;     // 功能寄存器输入
       List<bool> funRegisterOutputs;    // 功能寄存器输出
       List<bool> boolRegisterInputs;    // bool寄存器输入
       List<bool> boolRegisterOutputs;   // bool寄存器输出
       List<short> wordRegisterInputs;    // word寄存器输入
       List<short> wordRegisterOutputs;   // word寄存器输出
       List<double> floatRegisterInputs;   // float寄存器输入
       List<double> floatRegisterOutputs;  // float寄存器输出
   };

   // 机器人点动相关信息
   public struct MoveJogTaskParam
   {
       int jog_direction;  // 运动方向, -1:负方向, 1:正方向
       int jog_type;       // 1: 空间点动, 2: 关节点动
       int axis_num;       // 关节索引号
       double vel;             // 速度百分比
       int jog_coordinate; // 参考坐标系, 0: 世界, 1: 基座, 2: 工具, 3: 工件
       bool use_step;          // 是否步进模式
       double step_jointValue; // 关节步进距离, 单位:m, rad
       double step_cartvalue;  // 末端步进距离, 单位:m, rad
   };

   // 可达性检测相关信息
   public struct ReachabilityParam
   {
       bool result;                                  // 可达性确认结果
       List<List<double>> joints_pos; // 可达性检测成功时所对应的所有关节位置
   };

  // 外部轴反馈信息
  public struct EAxissInfo
  {
      string scheme_name;    // 外部轴方案名称
      int status;             // 激活状态
      double pos;                 // 当前位置
  };

     // 实时数据
    public struct RealTimeControlData
     {
         List<double> joint_pos_cmd;               // 关节位置
         List<double> joint_vel_cmd;               // 关节速度
         List<double> joint_torq_cmd;              // 关节力矩  预留
         List<double> cart_pos_tool_wobj_cmd;      // 笛卡尔位置
         List<double> cart_vel_tool_wobj_cmd;      // 笛卡尔速度
         List<double> cart_ft_cmd;                 // 笛卡尔力和力矩  预留
         bool status;                                      // 控制指令刷新状态, 更新机器人控制指令是给true
     };
  */

    public class DucoCobot
    {
        /**
         * @brief 创建机器人远程连接的实例
         * @param ip 机器人IP
         * @param port 机器人端口(7003)
         */
        public DucoCobot(string ip, int port, int time = 300000)
        {
            ip_ = ip;
            port_ = port;
            socket_ = new TSocket(ip_, port_, time);
            transport_ = new TBufferedTransport(socket_);
            protocol_ = new TBinaryProtocol(transport_);
            client_ = new RPCRobot.Client(protocol_);
        }
        /**
         * @brief 连接机器人
         * @return -1:打开失败; 0:打开成功
         */
        public int open()
        {
            try
            {
                transport_.Open();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }

            return 0;
        }
        /**
         * @brief 断开机器人连接
         * @return -1:打开失败; 0:打开成功
         */
        public int close()
        {
            try
            {
                transport_.Close();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }

            return 0;
        }
        /**
         * @brief 机器人上电
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int power_on(bool block)
        {
            try
            {
                return client_.power_on(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 机器人下电
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int power_off(bool block)
        {
            try
            {
                return client_.power_off(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 机器人上使能
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int enable(bool block)
        {
            try
            {
                return client_.enable(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 机器人下使能
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int disable(bool block)
        {
            try
            {
                return client_.disable(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 机器人关机
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int shutdown(bool block)
        {
            try
            {
                return client_.shutdown(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 停止所有任务
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int stop(bool block)
        {
            try
            {
                return client_.stop(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 暂停所有任务
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int pause(bool block)
        {
            try
            {
                return client_.pause(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 恢复所有暂停的任务
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int resume(bool block)
        {
            try
            {
                return client_.resume(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 运行程序脚本
         * @param name  脚本程序名称
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int run_program(string name, bool block)
        {
            try
            {
                return client_.run_program(name, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 设置工具末端相对于法兰面坐标系的偏移, 设置成功后, 
	     *        后续运动函数中的TCP设置为该TCP或者为空时,使用该TCP参数
         * @param name           工具坐标系的名字    
         * @param tool_offset    工具TCP偏移量 [x_off, y_off,z_off,rx,ry,rz], 单位: m, rad
         * @param payload        末端负载质量, 质心, [mass,x_cog,y_cog,z_cog], 单位: kg, m
         * @param inertia_tensor 末端工具惯量矩阵参数, 参数1-6分别对应矩阵xx、xy、xz、yy、yz、zz元素, 单位: kg*m^2
         * @return 返回当前任务结束时的状态
         */
        public int set_tool_data(string name, List<double> tool_offset, List<double> payload, List<double> inertia_tensor)
        {
            try
            {
                return client_.set_tool_data(name, tool_offset, payload, inertia_tensor);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 获取当前设置工具的负载质量及质心位置
         * @param _return 质量单位: kg,质心位置单位: m,[mass,x_cog,y_cog,z_cog]
         */
        public List<double> get_tool_load()
        {
            try
            {
                return client_.get_tool_load();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 获取当前状态下机械臂有效的末端工具的偏移量
         * @param _return TCP偏移量信息,单位: m, rad
         */
        public List<double> get_tcp_offset()
        {
            try
            {
                return client_.get_tcp_offset();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 获取当前设置的工件坐标系的值
         * @param _return [x, y, z, rx, ry, rz]工件坐标系相对于基坐标系的位移, 单位: m, rad
         */
        public List<double> get_wobj()
        {
            try
            {
                return client_.get_wobj();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 设置工件坐标系(请尽量使用set_wobj_offset)
         * @param name 工件坐标系名称
         * @param wobj 工件坐标系
         * @return 返回当前任务结束时的状态
         */
        public int set_wobj(string name, List<double> wobj)
        {
            try
            {
                return client_.set_wobj(name, wobj);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 基于当前的工件坐标系设置一个偏移量, 后续的move类脚本的参考工件坐标系上都将添加这个偏移量
         * @param wobj 工件坐标系相对于基坐标系的位移, 单位: m, rad
         * @param active 是否启用
         * @return 返回当前任务结束时的状态
         */
        public int set_wobj_offset(List<double> wobj, bool active = true)
        {
            try
            {
                return client_.set_wobj_offset(wobj, active);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 计算机械臂的正运动学, 求解给定TCP在给定wobj下的值
         * @param _return            末端姿态列表[x,y,z,rx,ry,rz]
         * @param joints_position 需要计算正解的关节角, 单位: rad
         * @param tool            工具坐标系信息,tcp偏移量[x_off,y_off,z_off,rx,ry,rz], 单位:m, rad, 为空使用当前tcp值
         * @param wobj            工件坐标系相对于基坐标系的位移[x, y, z, rx, ry, rz], 单位:m, rad, 为空使用当前wobj
         */
        public List<double> cal_fkine(List<double> joints_position, List<double> tool, List<double> wobj)
        {
            try
            {
                return client_.cal_fkine(joints_position, tool, wobj);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 计算运动学逆解, 在求解过程中, 会选取靠近当前机械臂关节位置的解
         * @param _return   关节位置列表[q1,q2,q3,q4,q5,q6]
         * @param p      需要计算的末端位姿在设置工件坐标系的值,包含当前有效的工具偏移量, 单位:m,rad
         * @param q_near 用于计算逆运动学的参考关节位置,为空使用当前关节值
         * @param tool   工具坐标系信息,tcp偏移量[x_off,y_off,z_off,rx,ry,rz], 单位:m, rad, 为空使用当前tcp值
         * @param wobj   工件坐标系相对于基坐标系的位移[x, y, z, rx, ry, rz], 单位:m, rad, 为空使用当前wobj
         */
        public List<double> cal_ikine(List<double> p, List<double> q_near, List<double> tool, List<double> wobj)
        {
            try
            {
                return client_.cal_ikine(p, q_near, tool, wobj);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 该函数可控制控制柜上的IO输出口的高低电平
         * @param num   控制柜上的IO输出口序号, 范围从1-16
         * @param value true为高电平, false为低电平
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int set_standard_digital_out(short num, bool @value, bool block)
        {
            try
            {
                return client_.set_standard_digital_out(num, value, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief set_tool_digital_out
         * @param num   机械臂末端的IO输出口序号, 范围从1-2
         * @param value true为高电平, false为低电平
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int set_tool_digital_out(short num, bool @value, bool block)
        {
            try
            {
                return client_.set_tool_digital_out(num, value, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 读取控制柜上的用户IO输入口的高低电平, 返回true为高电平, false为低电平
         * @param num 控制柜上的IO输入口序号, 范围从1-16
         * @return true为高电平, false为低电平
         */
        public bool get_standard_digital_in(short num)
        {
            try
            {
                return client_.get_standard_digital_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 获取控制柜上通用IO输出口的高低电平, 返回true为高电平, false为低电平
         * @param num 控制柜上的IO输出口序号, 范围从1-16
         * @return true为高电平, false为低电平
         */
        public bool get_standard_digital_out(short num)
        {
            try
            {
                return client_.get_standard_digital_out(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
         * @param num 机械臂末端的IO输出口序号, 范围从1-2
         * @return true为高电平, false为低电平
         */
        public bool get_tool_digital_in(short num)
        {
            try
            {
                return client_.get_tool_digital_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
         * @param num 机械臂末端的IO输出口序号, 范围从1-2
         * @return true为高电平, false为低电平
         */
        public bool get_tool_digital_out(short num)
        {
            try
            {
                return client_.get_tool_digital_out(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool get_config_digital_in(short num)
        {
            try
            {
                return client_.get_config_digital_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 读取控制柜上的模拟电压输入
         * @param num 控制柜上的模拟电压通道序号, 范围从1-4
         * @return 对应通道的模拟电压值
         */
        public double get_standard_analog_voltage_in(short num)
        {
            try
            {
                return client_.get_standard_analog_voltage_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 读取机械臂末端的模拟电压输入
         * @param num 机械臂末端的模拟电压通道序号, 范围从1-2
         * @return 对应通道的模拟电压值
         */
        public double get_tool_analog_voltage_in(short num)
        {
            try
            {
                return client_.get_tool_analog_voltage_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 读取控制柜上的模拟电流输入
         * @param num 控制柜上的模拟电流通道序号, 范围从1-4
         * @return 对应通道的模拟电流值
         */
        public double get_standard_analog_current_in(short num)
        {
            try
            {
                return client_.get_standard_analog_current_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int set_standard_analog_voltage_out(short num, double @value, bool block)
        {
            try
            {
                return client_.set_standard_analog_voltage_out(num, value, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int set_standard_analog_current_out(short num, double @value, bool block)
        {
            try
            {
                return client_.set_standard_analog_current_out(num, value, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public List<sbyte> read_data_485()
        {
            try
            {
                return client_.read_data_485();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<sbyte> read_raw_data_485(int len)
        {
            try
            {
                return client_.read_raw_data_485(len);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<sbyte> read_raw_data_485_ht(List<sbyte> head, List<sbyte> tail)
        {
            try
            {
                return client_.read_raw_data_485_ht(head, tail);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<sbyte> read_raw_data_485_h(List<sbyte> head, int len)
        {
            try
            {
                return client_.read_raw_data_485_h(head, len);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public bool write_data_485(List<sbyte> data)
        {
            try
            {
                return client_.write_data_485(data);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool write_raw_data_485(List<sbyte> data)
        {
            try
            {
                return client_.write_raw_data_485(data);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool write_raw_data_485_h(List<sbyte> data, List<sbyte> head)
        {
            try
            {
                return client_.write_raw_data_485_h(data, head);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool write_raw_data_485_ht(List<sbyte> data, List<sbyte> head, List<sbyte> tail)
        {
            try
            {
                return client_.write_raw_data_485_ht(data, head, tail);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public List<sbyte> tool_read_data_485()
        {
            try
            {
                return client_.tool_read_data_485();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<sbyte> tool_read_raw_data_485(int len)
        {
            try
            {
                return client_.tool_read_raw_data_485(len);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<sbyte> tool_read_raw_data_485_h(List<sbyte> head, int len)
        {
            try
            {
                return client_.tool_read_raw_data_485_h(head, len);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }

        }
        public List<sbyte> tool_read_raw_data_485_ht(List<sbyte> head, List<sbyte> tail)
        {
            try
            {
                return client_.tool_read_raw_data_485_ht(head, tail);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public bool tool_write_data_485(List<sbyte> data)
        {
            try
            {
                return client_.tool_write_data_485(data);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool tool_write_raw_data_485(List<sbyte> data)
        {
            try
            {
                return client_.tool_write_raw_data_485(data);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }

        }
        public bool tool_write_raw_data_485_h(List<sbyte> data, List<sbyte> head)
        {
            try
            {
                return client_.tool_write_raw_data_485_h(data, head);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool tool_write_raw_data_485_ht(List<sbyte> data, List<sbyte> head, List<sbyte> tail)
        {
            try
            {
                return client_.tool_write_raw_data_485_ht(data, head, tail);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }

        }
        public List<sbyte> read_data_can()
        {
            try
            {
                return client_.read_data_can();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<sbyte> read_raw_data_can()
        {
            try
            {
                return client_.read_raw_data_can();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public bool write_data_can(int id, List<sbyte> data)
        {
            try
            {
                return client_.write_data_can(id, data);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool write_raw_data_can(int id, List<sbyte> data)
        {
            try
            {
                return client_.write_raw_data_can(id, data);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool get_function_digital_in(short num)
        {
            try
            {
                return client_.get_function_digital_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool get_function_digital_out(short num)
        {
            try
            {
                return client_.get_function_digital_out(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public bool read_bool_reg(short num)
        {
            try
            {
                return client_.read_bool_reg(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        public int read_word_reg(short num)
        {
            try
            {
                return client_.read_word_reg(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public double read_float_reg(short num)
        {
            try
            {
                return client_.read_float_reg(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int write_bool_reg(short num, bool @value)
        {
            try
            {
                return client_.write_bool_reg(num, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int write_word_reg(short num, int @value)
        {
            try
            {
                return client_.write_word_reg(num, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int write_float_reg(short num, double @value)
        {
            try
            {
                return client_.write_float_reg(num, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public bool get_function_reg_in(short num)
        {
            try
            {
                return client_.get_function_reg_in(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        bool get_function_reg_out(short num)
        {
            try
            {
                return client_.get_function_reg_out(num);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态
     * @param joints_list 1-6关节的目标关节角度, 单位: rad
     * @param v 关节角速度, 单位: 系统设定速度的百分比%, 取值范围(0,100]
     * @param a 关节角加速度, 单位: 系统设定加速度的百分比%, 取值范围(0,100]
     * @param r 关节融合半径, 单位: 系统设定最大融合半径的百分比%, 默认值为 0, 表示无融合, 取值范围[0,50)
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int movej(List<double> joints_list, double v, double a, double r, bool block, OP op, bool def_acc = true)
        {
            Op op_ = ChangeParams(op);
            try
            {
                return client_.movej(joints_list, v, a, r, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态
     * @param joints_list 1-6关节的目标关节角度, 单位: rad
     * @param v 关节角速度, 单位: rad/s
     * @param a 关节角加速度, 单位: rad/s^2
     * @param r 关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int movej_pose(List<double> p, double v, double a, double r, List<double> q_near, string tool, string wobj, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movej_pose(p, v, a, r, q_near, tool, wobj, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 关节角速度, 单位: 系统设定速度的百分比%, 取值范围(0,100]
     * @param a 关节加速度, 单位: 系统设定加速度的百分比%, 取值范围(0,100]
     * @param r 关节融合半径, 单位: 系统设定最大融合半径的百分比%, 默认值为 0, 表示无融合, 取值范围[0,50)
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int movej2(List<double> joints_list, double v, double a, double r, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movej2(joints_list, v, a, r, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 关节角速度, 单位: rad/s
     * @param a 关节加速度, 单位: rad/s^2
     * @param r 关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int movej_pose2(List<double> p, double v, double a, double r, List<double> q_near, string tool, string wobj, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movej_pose2(p, v, a, r, q_near, tool, wobj, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂末端从当前状态按照直线路径移动到目标状态
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 末端速度, 单位: m/s
     * @param a 末端加速度, 单位: m/s^2
     * @param r 关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int movel(List<double> p, double v, double a, double r, List<double> q_near, string tool, string wobj, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movel(p, v, a, r, q_near, tool, wobj, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂做圆弧运动, 起始点为当前位姿点, 途径p1点, 终点为p2点
     * @param p1 圆弧运动中间点
     * @param p2 圆弧运动结束点
     * @param v  末端速度, 单位: m/s
     * @param a  末端加速度, 单位: m/s^2
     * @param r  关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param mode   姿态控制模式  0:姿态与终点保持一致;1:姿态与起点保持一致;2:姿态受圆心约束
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int movec(List<double> p1, List<double> p2, double v, double a, double r, int mode, List<double> q_near, string tool, string wobj, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movec(p1, p2, v, a, r, mode, q_near, tool, wobj, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂做圆周运动, 起始点为当前位姿点, 途径p1点和p2点
     * @param p1 圆周运动经过点
     * @param p2 圆周运动经过点
     * @param v  末端速度, 单位: m/s
     * @param a  末端加速度, 单位: m/s^2
     * @param r  关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param mode   姿态控制模式   1:姿态与终点保持一致;  2:姿态受圆心约束
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int move_circle(List<double> p1, List<double> p2, double v, double a, double r, int mode, List<double> q_near, string tool, string wobj, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.move_circle(p1, p2, v, a, r, mode, q_near, tool, wobj, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机械臂沿工具坐标系直线移动一个增量
     * @param pose_offset 工具坐标系下的位姿偏移量
     * @param v 直线移动的速度, 单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a 加速度, 单位: m/s^2
     * @param r 关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int tcp_move(List<double> pose_offset, double v, double a, double r, string tool, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.tcp_move(pose_offset, v, a, r, tool, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 控制机器人沿工具坐标系直线移动一个增量, 增量为p1与p2点之间的差, 运动的目标点为:当前点*p1^-1*p2
     * @param p1 工具坐标系下的位姿偏移量计算点1
     * @param p2 工具坐标系下的位姿偏移量计算点2
     * @param v  直线移动的速度, 单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a  加速度, 单位: m/s^2
     * @param r  关节融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int tcp_move_2p(List<double> p1, List<double> p2, double v, double a, double r, string tool, string wobj, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.tcp_move_2p(p1, p2, v, a, r, tool, wobj, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
     * @brief 样条运动函数, 控制机器人按照空间样条进行运动
     * @param pose_list 在设置工件坐标系下的末端位姿列表,最多不超过50个点
     * @param v 末端速度, 单位: m/s
     * @param a 末端加速度, 单位: m/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合
     * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
        public int spline(List<List<double>> pose_list, double v, double a, string tool, string wobj, bool block, OP op, double r = 0, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.spline(pose_list, v, a, tool, wobj, block, op_, r, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int speedj(List<double> joints_list, double a, int time, bool block)
        {
            try
            {
                return client_.speedj(joints_list, a, time, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int speedl(List<double> pose_list, double a, int time, bool block)
        {
            try
            {
                return client_.speedl(pose_list, a, time, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int speed_stop(bool block)
        {
            try
            {
                return client_.speed_stop(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int servoj(List<double> joints_list, double v, double a, bool block, double kp, double kd)
        {
            try
            {
                return client_.servoj(joints_list, v, a, block, kp, kd);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int servoj_pose(List<double> pose_list, double v, double a, List<double> q_near, string tool, string wobj, bool block, double kp, double kd)
        {
            try
            {
                return client_.servoj_pose(pose_list, v, a, q_near, tool, wobj, block, kp, kd);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int servo_tcp(List<double> pose_offset, double v, double a, string tool, bool block, double kp, double kd)
        {
            try
            {
                return client_.servo_tcp(pose_offset, v, a, tool, block, kp, kd);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int teach_mode(bool block)
        {
            try
            {
                return client_.teach_mode(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int end_teach_mode(bool block)
        {
            try
            {
                return client_.end_teach_mode(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int modbus_add_signal(string ip, int slave_number, int signal_address, int signal_type, string signal_name)
        {
            try
            {
                return client_.modbus_add_signal(ip, slave_number, signal_address, signal_type, signal_name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int modbus_delete_signal(string signal_name)
        {
            try
            {
                return client_.modbus_delete_signal(signal_name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int modbus_read(string signal_name)
        {
            try
            {
                return client_.modbus_read(signal_name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int modbus_write(string signal_name, int @value)
        {
            try
            {
                return client_.modbus_read(signal_name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public void modbus_set_frequency(string signal_name, int frequence)
        {
            try
            {
                client_.modbus_set_frequency(signal_name, frequence);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
            }
        }
        public List<string> get_last_error()
        {
            try
            {
                return client_.get_last_error();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                str_temp_.Clear();
                return str_temp_;
            }
        }
        public int get_noneblock_taskstate(int id)
        {
            try
            {
                return client_.get_noneblock_taskstate(id);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public void log_info(string message)
        {
            try
            {
                client_.log_info(message);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
            }
        }
        public void log_error(string message)
        {
            try
            {
                client_.log_error(message);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
            }
        }
        public int simulation(bool sim, bool block)
        {
            try
            {
                return client_.simulation(sim, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int speed(double val)
        {
            try
            {
                return client_.speed(val);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public List<sbyte> get_robot_state()
        {
            try
            {
                return client_.get_robot_state();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                byte_temp_.Clear();
                return byte_temp_;
            }
        }
        public List<double> get_flange_pose()
        {
            try
            {
                return client_.get_flange_pose();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_flange_speed()
        {
            try
            {
                return client_.get_flange_speed();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_flange_acceleration()
        {
            try
            {
                return client_.get_flange_acceleration();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_tcp_pose()
        {
            try
            {
                return client_.get_tcp_pose();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_tcp_speed()
        {
            try
            {
                return client_.get_tcp_speed();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_tcp_acceleration()
        {
            try
            {
                return client_.get_tcp_acceleration();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_tcp_force()
        {
            try
            {
                return client_.get_tcp_force();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_actual_joints_position()
        {
            try
            {
                return client_.get_actual_joints_position();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_target_joints_position()
        {
            try
            {
                return client_.get_target_joints_position();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_actual_joints_speed()
        {
            try
            {
                return client_.get_actual_joints_speed();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_target_joints_speed()
        {
            try
            {
                return client_.get_target_joints_speed();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_actual_joints_acceleration()
        {
            try
            {
                return client_.get_actual_joints_acceleration();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_target_joints_acceleration()
        {
            try
            {
                return client_.get_target_joints_acceleration();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_actual_joints_torque()
        {
            try
            {
                return client_.get_actual_joints_torque();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public List<double> get_target_joints_torque()
        {
            try
            {
                return client_.get_target_joints_torque();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        public int stop_record_track()
        {
            try
            {
                return client_.stop_record_track();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int start_record_track(string name, int mode, string tool, string wobj, double interval)
        {
            try
            {
                return client_.start_record_track(name, mode, tool, wobj, interval);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int collision_detect(int @value)
        {
            try
            {
                return client_.collision_detect(value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 对记录的轨迹基于关节空间(或基于笛卡尔空间)复现
         * @param name 轨迹名称
         * @param value 轨迹速度, (系统设定速度的百分比%), 取值范围(0,100]
         * @param mode 复现方式, 0:基于关节空间, 1:基于笛卡尔空间
         * @return 任务结束时状态
         */
        public int replay(string name, int @value, int mode)
        {
            try
            {
                return client_.replay(name, value, mode);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 设置抓取负载.可以在程序运行过程中设置机器人当前的负载(质量、质心)
         * @param value 末端工具抓取负载质量, 质心, {mass,x_cog,y_cog,z_cog}, 
	     *              相对于工具坐标系, 质量范围[0, 35], 单位: kg, m
         * @return 任务结束时状态
         */
        public int set_load_data(List<double> @value)
        {
            try
            {
                return client_.set_load_data(value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂开启末端力控.开启末端力控后所有运动函数除正常运动外,
	     *        会额外基于已配置的末端力控参数进行末端力控运动
         * @return 返回值代表当前任务的id信息
         */
        public int fc_start()
        {
            try
            {
                return client_.fc_start();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂退出末端力控
         * @return 当前任务的id信息
         */
        public int fc_stop()
        {
            try
            {
                return client_.fc_stop();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 修改并配置机器人末端力控参数
         * @param direction 6个笛卡尔空间方向末端力控开关, 开为true, 关为false
         * @param ref_ft 6个笛卡尔空间方向末端力控参考力, 范围[-1000, 1000], X/Y/Z方向单位: N, 
	                 RX/RY/RZ方向单位: Nm, 方向符号参考末端力控参考坐标系方向
         * @param damp  6个笛卡尔空间方向末端力控阻尼, 范围[-10000, 10000], 
	     *              X/Y/Z方向单位: N/(m/s), RX/RY/RZ方向单位: Nm/(°/s)
         * @param max_vel 6个笛卡尔空间方向末端力控最大调整速度, 范围[-5, 5], X/Y/Z方向单位: m/s, 
	     *                范围[-2*PI, 2*PI], RX/RY/RZ方向单位: rad/s
         * @param number_list 6个笛卡尔空间方向末端与环境接触力死区, 范围[-1000, 1000], 
	     *                     X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
         * @param toolname 设置使用的末端力控工具的名称, 默认为当前使用的工具
         * @param wobjname 设置使用的末端力控工件坐标系的名称, 默认为当前使用的工件坐标系
         * @param value 末端力控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
         * @return 当前任务的id信息
         */
        public int fc_config(List<bool> direction, List<double> ref_ft, List<double> damp, List<double> max_vel, List<double> number_list, string tool, string wobj, int @value)
        {
            try
            {
                return client_.fc_config(direction, ref_ft, damp, max_vel, number_list, tool, wobj, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂仅产生末端力控运动, 为阻塞型指令
         * @return 任务结束时状态
         */
        public int fc_move()
        {
            try
            {
                return client_.fc_move();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂在末端力控过程中进行力安全监控
         * @param direction 6个笛卡尔空间方向末端力安全监控开关, 开为true, 关为false
         * @param ref_ft 6个笛卡尔空间方向末端力安全监控参考力, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm, 
	     *               方向符号参考末端力安全监控参考坐标系方向
         * @param toolname 设置使用的末端力安全监控工具的名称
         * @param wobjname 设置使用的末端力安全监控工件坐标系的名称
         * @param type 末端力安全监控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
         * @param force_property 监控力属性, 0为末端负载力及外力, 1为末端外力(不含负载),可缺省, 默认为0
         * @return 当前任务的id信息
         */
        public int fc_guard_act(List<bool> direction, List<double> ref_ft, string tool, string wobj, int type, int force_property = 0)
        {
            try
            {
                return client_.fc_guard_act(direction, ref_ft, tool, wobj, type, force_property);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂在末端力控过程中禁用力安全监控
         * @return 当前任务的id信息
         */
        public int fc_guard_deact()
        {
            try
            {
                return client_.fc_guard_deact();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂末端力传感器读数设置为指定值
         * @param direction 6个末端力传感器输出力设置标志位, 需要设置为true, 不需要设置为false
         * @param ref_ft 6个末端力传感器输出力设置目标值, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
         * @return 当前任务的id信息
         */
        public int fc_force_set_value(List<bool> direction, List<double> ref_ft)
        {
            try
            {
                return client_.fc_force_set_value(direction, ref_ft);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定位置判断条件时自动停止当前运动函数并调过后续运动函数, 
	     *        直到fc_stop()函数被执行停止末端力控
         * @param middle 位置判断条件绝对值, X/Y/Z方向单位: m, RX/RY/RZ方向单位: rad
         * @param range  位置判断条件偏移范围大小, X/Y/Z方向单位: m, RX/RY/RZ方向单位: rad
         * @param absolute 绝对/增量条件判断标志位, true为绝对位置判断, false为增量位置判断
         * @param duration 条件满足触发保持时间, 单位: ms
         * @param timeout  条件满足触发超时时间, 单位: ms
         * @return 当前任务的id信息
         */
        public int fc_wait_pos(List<double> middle, List<double> range, bool absolute, int duration, int timeout)
        {
            try
            {
                return client_.fc_wait_pos(middle, range, absolute, duration, timeout);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定速度判断条件时自动停止当前运动函数并跳过后续运动函数, 
	     *        直到fc_stop()函数被执行停止末端力控
         * @param middle 速度判断条件绝对值, X/Y/Z方向范围[-5, 5], 单位: m/s, RX/RY/RZ方向范围[-2*PI, 2*PI], 单位: rad/s
         * @param range  速度判断条件偏移范围大小, X/Y/Z方向单位: m/s, RX/RY/RZ方向单位: rad/s
         * @param absolute 绝对/增量条件判断标志位, true为绝对速度判断, false为增量速度判断
         * @param duration 条件满足触发保持时间, 单位: ms
         * @param timeout  条件满足触发超时时间, 单位: ms
         * @return 当前任务的id信息
         */
        public int fc_wait_vel(List<double> middle, List<double> range, bool absolute, int duration, int timeout)
        {
            try
            {
                return client_.fc_wait_vel(middle, range, absolute, duration, timeout);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定力判断条件时自动停止当前运动函数并跳过后续运动函数, 
	     *        直到fc_stop()函数被执行停止末端力控
         * @param middle 力判断条件绝对值, 范围[-1000, 1000], X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
         * @param range  力判断条件偏移范围大小, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
         * @param absolute 绝对/增量条件判断标志位, true为绝对力判断, false为增量力判断
         * @param duration 条件满足触发保持时间, 单位: ms
         * @param timeout  条件满足触发超时时间, 单位: ms
         * @return 当前任务的id信息
         */
        public int fc_wait_ft(List<double> middle, List<double> range, bool absolute, int duration, int timeout)
        {
            try
            {
                return client_.fc_wait_ft(middle, range, absolute, duration, timeout);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中位置条件判断、速度条件判断与力条件判断间的逻辑关系.不配置时默认三个条件判断都禁用
         * @param value 三维整形列表, 0代表不启用, 1代表与逻辑, 2代表或逻辑.例如开启位置条件判断, 禁用速度条件判断, 开启力条件判断, 并且位置与力的关系为或, 则输入[1,0,2]
         * @return 当前任务的id信息
         */
        public int fc_wait_logic(List<int> @value)
        {
            try
            {
                return client_.fc_wait_logic(value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 获取当前机器人末端传感器的反馈读数
         * @param _return 6自由度末端力读数, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
         */
        public List<double> fc_get_ft()
        {
            try
            {
                return client_.fc_get_ft();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 获取当前机器人末端力控功能启用状态
         * @return 机器人末端力控启用返回true, 未启用返回false
         */
        public bool fc_mode_is_active()
        {
            try
            {
                return client_.fc_mode_is_active();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 控制机械臂开启速度优化功能.开启该功能后, 在满足系统约束前提下,
	     *        机械臂以尽可能高的速度跟踪路径
         * @return 当前任务结束时的状态
         */
        public int enable_speed_optimization()
        {
            try
            {
                return client_.enable_speed_optimization();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂退出速度优化
         * @return 当前任务结束时的状态
         */
        public int disable_speed_optimization()
        {
            try
            {
                return client_.disable_speed_optimization();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public int set_system_value_bool(string name, bool @value)
        {
            try
            {
                return client_.set_system_value_bool(name, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public int set_system_value_double(string name, double @value)
        {
            try
            {
                return client_.set_system_value_double(name, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public int set_system_value_str(string name, string @value)
        {
            try
            {
                return client_.set_system_value_str(name, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public int set_system_value_list(string name, List<double> @value)
        {
            try
            {
                return client_.set_system_value_list(name, value);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public bool get_system_value_bool(string name)
        {
            try
            {
                return client_.get_system_value_bool(name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public double get_system_value_double(string name)
        {
            try
            {
                return client_.get_system_value_double(name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public string get_system_value_str(string name)
        {
            try
            {
                return client_.get_system_value_str(name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return "";
            }
        }
        /**
         * @brief 被废弃
         * @param name
         * @param value
         * @return
         */
        public List<double> get_system_value_list(string name)
        {
            try
            {
                return client_.get_system_value_list(name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                temp_.Clear();
                return temp_;
            }
        }
        /**
         * @brief 将一组points点位信息输入到机器人控制器中的轨迹池
         * @param track 一组points点位信息.每个point以6个double类型数据构成
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int trackEnqueue(List<List<double>> track, bool block)
        {
            try
            {
                return client_.trackEnqueue(track, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 将机器人控制器中的轨迹池清空
         * @return 当前任务结束时的状态
         */
        public int trackClearQueue()
        {
            try
            {
                return client_.trackClearQueue();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 获取机器人控制器中的当前轨迹池大小
         * @return 当前轨迹池大小
         */
        public int getQueueSize()
        {
            try
            {
                return client_.getQueueSize();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 执行时, 机器人的各关节将顺序到达轨迹池中的点位值直到轨迹池中无新的点位.
         *        执行过程中, 主导关节(关节位置变化最大的关节)将以speed与acc规划运动, 其他关节按比例缩放.
         *        注:如果已经开始执行停止规划, 将不再重新获取轨迹池中的数据, 直到完成停止.
         *            停止后如果轨迹池中有新的点位, 将重新执行跟随.为保证运动连续性, 建议至少保证轨迹池中有10个数据
         * @param speed 最大关节速度, 单位: rad/s
         * @param acc   最大关节加速度, 单位: rad/s^2
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int trackJointMotion(double speed, double acc, bool block)
        {
            try
            {
                return client_.trackJointMotion(speed, acc, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 执行时, 机器人的工具末端tool将顺序到达轨迹池中的点位值直到轨迹池中无新的点位.
         *        执行过程中, 工具末端tool将以speed与acc在工件坐标系wobj下规划运动.
         *        注:如果已经开始执行停止规划, 将不再重新获取轨迹池中的数据, 直到完成停止.
                  停止后如果轨迹池中有新的点位, 将重新执行跟随.为保证运动连续性, 建议至少保证轨迹池中有10个数据
         * @param speed 最大末端速度, 单位: m/s
         * @param acc   最大末端加速度, 单位: m/s^2
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @param tool  设置使用的工件坐标系的名称, 为空字符串时默认为当前使用的工件坐标系
         * @param wobj  设置使用的工具的名称,为空字符串时默认为当前使用的工具
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int trackCartMotion(double speed, double acc, bool block, string tool, string wobj, double radius)
        {
            try
            {
                return client_.trackCartMotion(speed, acc, block, tool, wobj, radius);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 确保机器人远程连接断开时, 机器人自动产生一条stop指令以停止当前运动.使用该函数需要单独创建一个线程周期性调用
         * @param time 心跳延时时间, 单位: ms
         */
        public void rpc_heartbeat(int time)
        {
            try
            {
                client_.rpc_heartbeat(time);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
            }
        }
        /**
         * @brief 控制机械臂开启加速度优化功能.开启该功能后, 系统会根据机器人动力学模型、电功率模型计算得到最优加速度大小,
         *        在满足速度约束前提下, 机械臂以尽可能高的加速度进行规划.当速度优化同时打开后, 该函数不起作用
         * @return 当前任务结束时的状态
         */
        public int enable_acc_optimization()
        {
            try
            {
                return client_.enable_acc_optimization();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制机械臂退出加速度优化
         * @return 当前任务结束时的状态
         */
        public int disable_acc_optimization()
        {
            try
            {
                return client_.disable_acc_optimization();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 设置485的波特率
         * @param value 波特率
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int set_baudrate_485(int @value, bool block)
        {
            try
            {
                return client_.set_baudrate_485(value, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 设置CAN的波特率
         * @param value 波特率
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @return  阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int set_baudrate_can(int @value, bool block)
        {
            try
            {
                return client_.set_baudrate_can(value, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        public int set_analog_output_mode(short num, int mode, bool block)
        {
            try
            {
                return client_.set_analog_output_mode(num, mode, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 判断机器人是否在运动
         * @return True:机器人在运动, False:机器人没有运动
         */
        public bool robotmoving()
        {
            try
            {
                return client_.robotmoving();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return false;
            }
        }
        /**
         * @brief 对多线圈进行写操作
         * @param slave_num modbus节点号
         * @param name modbus节点名
         * @param len 需要写入数据的线圈长度
         * @param byte_list 需要写入的数据
         * @return 任务结束时状态
         */
        public int modbus_write_multiple_coils(int slave_num, string name, int len, List<sbyte> byte_list)
        {
            try
            {
                return client_.modbus_write_multiple_coils(slave_num, name, len, byte_list);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }

        }
        /**
         * @brief 对多寄存器进行写操作
         * @param slave_num modbus节点号
         * @param name modbus节点名
         * @param len 需要写入数据的寄存器长度
         * @param word_list 需要写入的数据
         * @return 任务结束时状态
         */
        public int modbus_write_multiple_regs(int slave_num, string name, int len, List<short> word_list)
        {
            try
            {
                return client_.modbus_write_multiple_regs(slave_num, name, len, word_list);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 获取当前RPC库的版本号
         * @return 当前RPC库的版本号
         */
        public string get_version()
        {
            return "3.2.0";
        }

        public RobotStatus getRobotStatus()
        {
            return client_.getRobotStatus();
        }

        public IOStatus getRobotIOStatus()
        {
            return client_.getRobotIOStatus();
        }
        /**
         * @brief 获取当前工程的路径
         * @param project_path 当前工程路径
         */
        public string get_current_project()
        {
            try
            {
                return client_.get_current_project();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return "";
            }
        }

        public Dictionary<string, int> get_files_list(string path)
        {
            return client_.get_files_list(path);
        }
        /**
        * @brief 重置碰撞检测警告
        * @return 任务结束时状态
        */
        public int collision_detection_reset()
        {
            try
            {
                return client_.collision_detection_reset();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
		 * @brief 控制机械臂在关节或者笛卡尔空间做点动
		 * @param param Jog运动的相关参数, 参考MoveJogTaskParams
		 * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
		 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
		 */
        public int move_jog(MoveJogTaskParam param, bool block)
        {
            try
            {
                return client_.move_jog(param, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
		 * @brief 结束机械臂的关节或者笛卡尔Jog
		 * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
		 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
		 */
        public int stop_manual_move(bool block)
        {
            try
            {
                return client_.stop_manual_move(block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
		 * @brief 获取机器人控制器的软件版本号
		 * @param _return 机器人控制器的软件版本号
		 */
        string get_robot_version()
        {
            try
            {
                return client_.get_robot_version();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return "";
            }
        }
        /**
		 * @brief 启用或禁用示教器的物理按键
		 * @param enable true:启动示教器物理按键, false:禁用示教器物理按键
		 * @return 任务结束时状态
		 */
        public int set_teach_pendant(bool enable)
        {
            try
            {
                return client_.set_teach_pendant(enable);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
		 * @brief 获取示教速度的百分比
		 * @return 示教速度的百分比
		 */
        public int get_teach_speed()
        {
            try
            {
                return client_.get_teach_speed();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
		 * @brief 获取全局速度的百分比
		 * @return 全局速度的百分比
		 */
        public int get_global_speed()
        {
            try
            {
                return client_.get_global_speed();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
		 * @brief 设置示教速度的百分比
		 * @param v 示教速度的百分比, 范围[1,100]
		 * @return 任务结束时状态
		 */
        public int set_teach_speed(int v)
        {
            try
            {
                return client_.set_teach_speed(v);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
        * @brief 设置复合运动的相关参数
        * @param type 复合运动类型.1:平面三角形轨迹, 2:平面正旋轨迹,
        *                           3:平面圆形轨迹, 4:平面梯形轨迹, 5:平面8字形轨迹
        * @param ref_plane  参考平面, 0:工具XOY, 1:工具XOZ
        * @param fq 频率, 单位: Hz
        * @param amp 振幅, 单位: m
        * @param el_offset 仰角偏移, 单位: m.(参数预留)
        * @param az_offset 方向角偏移, 单位: m.(参数预留)
        * @param up_height 中心隆起高度, 单位: m.(参数预留)
        * @param time 左右停留时间
        * @param op_list 二维的OP参数列表
        * @return 任务结束时状态
        */
        public int combine_motion_config(int type, int ref_plane, int fq, int amp, int el_offset, int az_offset, int up_height, List<int> time,  List<OP> op_list)
        {
            try
            {
                List<Op> op_tg = new List<Op>();
                for (int i = 0; i < op_list.Count; i++)
                {
                    Op op_ = ChangeParams(op_list[i]);
                    op_tg.Add(op_);
                }
                return client_.combine_motion_config(type, ref_plane, fq, amp, el_offset, az_offset, up_height, time, op_tg);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
		 * @brief 开启复合运动
		 * @return 任务结束时状态
		 */
        public int enable_combine_motion()
        {
            try
            {
                return client_.enable_combine_motion();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
 * @brief 结束复合运动
 * @return 任务结束时状态
 */
        public int disable_combine_motion()
        {
            try
            {
                return client_.disable_combine_motion();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 开启机械臂奇异点规避功能
         * @return 任务结束时状态
         */
        public int enable_singularity_control()
        {
            try
            {
                return client_.enable_singularity_control();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
        * @brief 关闭机械臂奇异点规避功能
        * @return 任务结束时状态
        */
        public int disable_singularity_control()
        {
            try
            {
                return client_.disable_singularity_control();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 开启末端震动抑制功能
         * @return 任务结束时状态
         */
        public int enable_vibration_control()
        {
            try
            {
                return client_.enable_vibration_control();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 关闭末端震动抑制功能
         * @return 任务结束时状态
         */
        public int disable_vibration_control()
        {
            try
            {
                return client_.disable_vibration_control();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 控制外部轴移动
         * @param scheme_name 目标外部轴方案名称
         * @param epose 目标外部轴方案所对应自由度位置(三维),
         *              记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
         * @param v     外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @param op    详见上方Op特殊类型说明(距离触发无效),可缺省参数
         * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
         *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
 */
        public int move_eaxis(string scheme_name, List<double> epose, double v, bool block, OP op)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.move_eaxis(scheme_name, epose, v, true, op_);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 控制外部轴和机器人执行关节运动
         * @param joints_list 目标关节位置, 单位: rad
         * @param v 关节角速度, 单位: rad/s
         * @param a 关节加速度, 单位: rad/s^2
         * @param rad  融合半径, 单位: m
         * @param scheme_name 目标外部轴方案名称
         * @param epose 目标外部轴方案所对应自由度位置(三维),
         *              记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
         * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
         * @param block   指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @param op      详见上方Op特殊类型说明(距离触发无效), 可缺省参数
         * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
         * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
         *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
         */
        public int movej2_eaxis(List<double> joints_list, double v, double a, double rad, string scheme_name, List<double> epose, double eaxis_v, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movej2_eaxis(joints_list, v, a, rad, scheme_name, epose, eaxis_v, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 控制外部轴和机器人从当前状态, 按照关节运动的方式移动到末端目标位置
         * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 范围[-2*PI, 2*PI], 单位: rad
         * @param v 关节角速度, 单位: rad/s
         * @param a 关节加速度, 单位: rad/s^2
         * @param rad 融合半径, 单位: m
         * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
         * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
         * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
         * @param scheme_name 目标外部轴方案名称
         * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
         * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
         * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
         * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
         *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
         */
        public int movej2_pose_eaxis(List<double> p, double v, double a, double rad, List<double> qnear, string tool, string wobj, string scheme_name, List<double> epose, double eaxis_v, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movej2_pose_eaxis(p, v, a, rad, qnear, tool, wobj, scheme_name, epose, eaxis_v, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 控制外部轴和机器人从当前状态按照直线路径移动到目标状态
         * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 范围[-2*PI, 2*PI], 单位: rad
         * @param v 末端速度, 范围(0, 5], 单位: m/s
         * @param a 末端加速度, 范围(0, ∞], 单位: m/s^2
         * @param rad 融合半径, 单位: m
         * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
         * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
         * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
         * @param scheme_name 目标外部轴方案名称
         * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
         * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
         * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
         * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
         *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
         */
        public int movel_eaxis(List<double> p, double v, double a, double rad, List<double> qnear, string tool, string wobj, string scheme_name, List<double> epose, double eaxis_v, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movel_eaxis(p, v, a, rad, qnear, tool, wobj, scheme_name, epose, eaxis_v, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 控制外部轴和机器人做圆弧运动, 起始点为当前位姿点, 途径p1点, 终点为p2点
         * @param p1 圆弧运动中间点位姿
         * @param p2 圆弧运动结束点位姿
         * @param v 末端速度, 范围(0, 5], 单位: m/s
         * @param a 末端加速度, 范围(0, ∞], 单位: m/s^2
         * @param rad 融合半径, 单位: m
         * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
         * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
         * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
         * @param scheme_name 目标外部轴方案名称
         * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
         * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
         * @param def_acc 是否使用自定义加速度, true表示使用自定义的加速度值, false表示使用系统自动规划的加速度值, 可缺省, 默认为true
         * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
         *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
         */
        public int movec_eaxis(List<double> p1, List<double> p2, double v, double a, double rad, List<double> qnear, string tool, string wobj, string scheme_name, List<double> epose, double eaxis_v, bool block, OP op, bool def_acc = true)
        {
            try
            {
                Op op_ = ChangeParams(op);
                return client_.movec_eaxis(p1, p2, v, a, rad, qnear, tool, wobj, scheme_name, epose, eaxis_v, block, op_, def_acc);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 可达性检查
         * @param _return 可达性确认结果
         * @param base 基坐标在世界坐标系中的位置
         * @param wobj 工件坐标系在世界坐标系中的位置
         * @param tool 工具坐标系在法兰坐标系中的描述
         * @param ref_pos 机器人关节参考角度
         * @param check_points 需要确认可达性检查的点位列表
         */
        ReachabilityParam reach_check(List<double> @base, List<double> wobj, List<double> tool, List<double> ref_pos, List<List<double>> check_points)
        {
            try
            {
                return client_.reach_check(@base, wobj, tool, ref_pos, check_points);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                ReachabilityParam tmp = new ReachabilityParam();
                return tmp;
            }
        }

        /**
         * @brief 控制外部轴和机器人执行点动
         * @param name 目标外部轴方案名称
         * @param direction 运动方向, -1:负方向, 1:正方向
         * @param vel 速度百分比
         * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
         */
        public int move_jog_eaxis(string name, int direction, double vel, bool block)
        {
            try
            {
                return client_.move_jog_eaxis(name, direction, vel, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 获取外部轴当前位置和激活状态信息
         * @param info 当前位置和激活状态信息
         */
        List<EAxissInfo> get_eaxis_info()
        {
            try
            {
                return client_.get_eaxis_info();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                List<EAxissInfo> tmp = new List<EAxissInfo>();
                return tmp;
            }
        }

        /**
         * @brief 启动外部轴方案
         * @param scheme_name 外部轴方案名称
         * @return 任务结束时的状态
         */
        public int enable_eaxis_scheme(string scheme_name)
        {
            try
            {
                return client_.enable_eaxis_scheme(scheme_name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 结束外部轴方案
         * @param scheme_name 外部轴方案名称
         * @return 任务结束时的状态
         */
        public int disable_eaxis_scheme(string scheme_name)
        {
            try
            {
                return client_.disable_eaxis_scheme(scheme_name);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 设置牵引时的参数
         * @param space        牵引类型
         * @param joint_scale  关节柔顺度
         * @param cart_scale   笛卡尔柔顺度
         * @param coord_type   类型
         * @param direction    牵引方向激活
         * @return 任务结束时的状态
         */
        public int set_hand_teach_parameter(int space, int joint_scale, int cart_scale, int coord_type, List<bool> direction)
        {
            try
            {
                return client_.set_hand_teach_parameter(space, joint_scale, cart_scale, coord_type, direction);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 示教器按键的jog类型
         * @param type 1:关节, 2:笛卡尔
         * @return 任务结束时的状态
         */
        public int set_pendant_type(int type)
        {
            try
            {
                return client_.set_pendant_type(type);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 设置融合预读取配置
         * @param per 百分比(%)
         * @return 任务结束时的状态
         */
        public int set_blend_ahead(int per)
        {
            try
            {
                return client_.set_blend_ahead(per);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 开启实时控制模式
         * @param mode  实时控制模式，1：关节位置，2：关节速度，3：空间位置，4：空间速度
         * @param filter_bandwidth 实时控制指令滤波器带宽，单位Hz，默认100Hz
         * @param com_lost_time 实时控制通讯数据丢失监控保护时间，单位s，默认0.02s
         * @return 任务结束时的状态
         */
        public int start_realtime_mode(int mode, double filter_bandwidth = 100, double com_lost_time = 0.02)
        {
            try
            {
                return client_.start_realtime_mode(mode, filter_bandwidth, com_lost_time);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 结束实时控制模式
         * @return 任务结束时的状态
         */
        public int end_realtime_mode()
        {
            try
            {
                return client_.end_realtime_mode();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        /**
         * @brief 实时数据入队
         * @param realtime_data 实时数据
         * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
         * @return 任务结束时的状态
         */
        public int realtime_data_enqueue(List<RealTimeControlData> realtime_data, bool block)
        {
            try
            {
                return client_.realtime_data_enqueue(realtime_data, block);
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 清空实时数据队列
         * @return 任务结束时的状态
         */
        public int clear_realtime_data_queue()
        {
            try
            {
                return client_.clear_realtime_data_queue();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }
        /**
         * @brief 获取当前实时队列池数据的数量
         * @return 当前实时队列池数据的数量
         */
        public int get_realtime_data_queue_size()
        {
            try
            {
                return client_.get_realtime_data_queue_size();
            }
            catch (Exception e)
            {
                error_msg_ = e.Message;
                return -1;
            }
        }

        private int port_;
        private string ip_;
        private TSocket socket_;
        private TTransport transport_;
        private TProtocol protocol_;
        private RPCRobot.Client client_;
        private OP op_ = new OP(0, 1, false, 0.0, 0.0, "", 0, 1, false, 0.0, 0.0, "");
        private static bool stop_send_ = false;
        private static EndPoint ip_port;
        private byte[] soi = new byte[2] { 0xFF, 0xFF };
        private byte[] eoi = new byte[2] { 0xEE, 0xEE };
        private byte[] bytes = new byte[44];
        private string error_msg_ = "";
        private List<double> temp_ = null;
        private List<sbyte> byte_temp_;
        private List<string> str_temp_;

        private Op ChangeParams(OP op)

        {
            Op op_ = new Op(op.time_or_dist_1, op.trig_io_1, op.trig_value_1, op.trig_time_1, op.trig_dist_1, op.trig_event_1,
                op.time_or_dist_2, op.trig_io_2, op.trig_value_2, op.trig_time_2, op.trig_dist_2, op.trig_event_2);

            return op_;
        }
    }
}
