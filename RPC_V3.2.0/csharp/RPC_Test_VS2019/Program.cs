using System;
using System.Collections.Generic;
using DucoRPC;


namespace DucoCobot_Demo
{
    class Program
    {
        static void Main(string[] args)
        {
            string ip = "192.168.1.10";
            int port = 7003;
            OP _op = new OP(4, 1, false, 0, 0, "", 0, 0, false, 0, 0, "");
            DucoCobot duco_cobot = new DucoCobot(ip, port);
            int rlt = duco_cobot.open();
            rlt = duco_cobot.power_on(true);
            Console.WriteLine("power on");
            Console.WriteLine( rlt);
            rlt = duco_cobot.enable(true);
            Console.WriteLine("enable");
            Console.WriteLine(rlt);
            List<double> pos = new List<double> { 0, 0, 0, 0, 0, 0 };
            rlt = duco_cobot.movej2(pos, 1, 1, 0, true, _op);
            Console.WriteLine("movej2");
            Console.WriteLine( rlt);
            List<double> pos2 = new List<double> { 0, 0, 1.57, 0, -1.57, 0 };
            rlt = duco_cobot.movej2(pos2, 1, 1, 0, true, _op);
            Console.WriteLine("movej2");
            Console.WriteLine(rlt);
            duco_cobot.log_info("test");
            rlt = duco_cobot.disable(true);
            Console.WriteLine("disable");
            Console.WriteLine(rlt);
            rlt = duco_cobot.power_off(true);
            Console.WriteLine("power off");
            Console.WriteLine(rlt);
            rlt = duco_cobot.close();

        }
    }
}
