from CalcTools import CalcTools
import math
import numpy as np
import tqdm
import time
import sys

def progress_bar(duration, bar_length=50):
    # duration: 进度条持续时间（秒）
    # bar_length: 进度条长度（字符数）
    
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        if elapsed_time > duration:
            break
        
        progress = elapsed_time / duration
        block = int(round(bar_length * progress))
        text = f"\rProgress: [{'#' * block + '-' * (bar_length - block)}] {int(progress * 100)}%"
        sys.stdout.write(text)
        sys.stdout.flush()
        
        time.sleep(0.1)  # 更新频率为0.1秒

    # 完成进度条
    text = f"\rProgress: [{'#' * bar_length}] 100%\n"
    sys.stdout.write(text)
    sys.stdout.flush()


def waiting_for_connections(duration):
    end_time = time.time() + duration
    dots = ['.', '..', '...']
    index = 0

    while time.time() < end_time:
        sys.stdout.write("\rWaiting for MainCtrlSystem's connection" + dots[index])
        sys.stdout.flush()
        index = (index + 1) % len(dots)
        time.sleep(0.5)  # 控制闪烁速度

    # 完成后清除进度条
    sys.stdout.write("\rWaiting for MainCtrlSystem's connections   \n")
    sys.stdout.flush()

if __name__ == '__main__':
   print('Auto Twist System is Loading ! ! !')
   progress_bar(5)


   print('Connecting Subsystems ! ! !')
   

   # print('Auto Twist System is loading ! ! !')
   print('Connecting Duco...')
   progress_bar(0.1)
   print('Duco is connected ! ! !')
   print(' ip: 192.168.1.16')
   print(' port: 7003\n')

   print('Connecting Danikor...')
   progress_bar(0.5)
   print('Danikor is connected ! ! !')
   print(' ip: 192.168.1.15')
   print(' port: 8888\n')

   print('Connecting HikCam...')
   progress_bar(0.8)
   print('HikCam is connected ! ! !')
   print(' ip: 192.168.1.3')
   print(' port: 8192\n')

   print('Connecting TransferCam')
   progress_bar(0.3)
   print('TransferCam is connected ! ! !')
   print(' ip: 192.168.1.10')
   print(' port: 5700\n')

   
   print('host\n ip: 192.168.1.225')
   print(' port: 9999')
   waiting_for_connections(10)

   print('MainCtrlSystem is connected ! ! !')
   print('Waiting for command ! ! !')


   print('Command:\n 1')

   print('\nmoving\n')
   # progress_bar(15)

   print('At the pos of GetScrewPhoto\n')

   
   print('detecting screw...')
   progress_bar(2)
   print('screw target pos\n     [0.9981,0.113515,0.03657,1.57267,0.000188,01.576911]')

   print('\nmoving\n')
   time.sleep(5)
   # progress_bar(15)

   print('Get screw')
   time.sleep(5)

   print('\nmoving\n')
   time.sleep(10)

   print('Screw is Ready ! ! ')
   time.sleep(0.5)

   print('\nmoving\n')
   time.sleep(20)

   print('At pos of TargetPhoto')
   time.sleep(0.5)

   print('target posture from TransferCam\n  [3.171592,0.000188,-1.576911]')
   time.sleep(0.5)
   
   
   print('detecting Target...')
   progress_bar(2)
   print('target pos\n     [0.326456,-1.311883,-0.151848,3.171592,0.000188,-1.576911]')
   time.sleep(0.5)

   print('\nmoving\n')
   time.sleep(15)


   print('Twisting...')
   progress_bar(5)

   print('Final Torque\n   8.16')
   print('Target Torque\n   8')
   print('Twist successed\n')

   print('Going to defaultPos...')
   print('\nmoving\n')
   time.sleep(15)
   
   print('System initialization')
   progress_bar(3)

   print('\nWaiting for command ! ! !')
   time.sleep(20)