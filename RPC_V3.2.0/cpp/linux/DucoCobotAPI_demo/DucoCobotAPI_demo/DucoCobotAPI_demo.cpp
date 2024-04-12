#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include<unistd.h>
#endif
#include "DucoCobot.h"

using namespace std;
// 添加DucoRPC命名空间
using namespace DucoRPC;

static std::string ip = "127.0.0.1";
static bool stopthread = false;


void thread_funA()
{
    // 多线程必须新创建DucoCobot对象
    DucoCobot duco_cobot_a(ip, 7003);
    try
    {
        duco_cobot_a.open();
        std::vector<double> joints;

        while (!stopthread)
        {
            joints.clear();
            // 获取关节位置
            duco_cobot_a.get_actual_joints_position(joints);
#ifdef _WIN32
            Sleep(1000);
#else
            usleep(1000);
#endif
        }
    }
    catch (...)
    {
        std::cout << "thread error!" << std::endl;
        duco_cobot_a.close();
    }

    duco_cobot_a.close();
}

void thread_funB()
{
    // 多线程必须新创建DucoCobot对象
    DucoCobot duco_cobot_b(ip, 7003);
    duco_cobot_b.open();
    int result = duco_cobot_b.stop(true);

    if(result == -1)
    {
        // -1 : SDK出现错误
        cout << "error" << endl;
    }

    duco_cobot_b.close();
}


int main(int argc, char *argv[])
{
    OP op;
    op.trig_io_1 = 5;
    op.trig_io_2 = 0;
    op.trig_dist_1 = 0;
    op.trig_dist_2 = 0;
    op.trig_time_1 = 3;
    op.trig_time_2 = 0;
    op.trig_value_1 = true;
    op.trig_value_2 = false;
    op.time_or_dist_1 = 1;
    op.time_or_dist_2 = 0;
    op.trig_event_1 = "";
    op.trig_event_2 = "";
    DucoCobot duco_cobot(ip, 7003);

    try
    {
        // 启动线程
        std::thread thread_state(thread_funA);
        // 建立网络连接
        int result = duco_cobot.open();
        cout << "open: " << result << endl;
        // 机器人上电
        result = duco_cobot.power_on(true);
        cout << "power_on: " << result << endl;
        // 机器人使能
        result = duco_cobot.enable(true);
        cout << "enable: " << result << endl;

        std::vector<double> joints = {0,0,1.570796,0,-1.570796,0};
        // 关节运动
        result = duco_cobot.movej2(joints, 1, 1, 0, true);
        cout << "joint2: " << result << endl;

        string project_path("");
        // 获取当前工程名称
        duco_cobot.get_current_project(project_path);
        cout << project_path << endl;

        map<string, int> files;
        project_path += "/program/";
        // 获取程序文件列表
        duco_cobot.get_files_list(files, project_path);
        map<string, int>::iterator itr;
        for (itr = files.begin(); itr != files.end(); ++itr)
        {
            cout << itr->first << " " << itr->second << endl;
        }

        // 运行test1.jspf程序  (非阻塞调用)
        result = duco_cobot.run_program("test.jspf", false);
        if(result != -1)
        {
            // 获取当前任务的状态
            bool stop_tmp = false;
            while(!stop_tmp)
            {
                switch(duco_cobot.get_noneblock_taskstate(result))
                {
                case 1:
                    usleep(10000);
                    break;
                case 4:
                    stop_tmp = true;
                    break;
                case 2:
                case 3:
                case 5:
                    duco_cobot.stop(true);
                    stop_tmp = true;
                    break;
                default:
                    cout << "error" << endl;
                    stop_tmp = true;
                    break;
                }
            }
        }

        stopthread = true;
        thread_state.join();
        sleep(1);
        duco_cobot.close();
    }
    catch (...)
    {
        std::cout << "main error!" << std::endl;
        duco_cobot.close();
    }
}

