
import time
import os,sys
import subprocess

sys.path.append('/home/lab/.local/lib/python3.8/site-packages/')
sys.path.append('/home/lab/kuavo/tools/check_tool/Ruierman/')

import serial
import serial.tools.list_ports
import dynamixel_servo


import ruierman
import claw_rs485


servo_usb_path = "/dev/usb_servo"
claw_usb_path = "/dev/claw_serial"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# print(bcolors.HEADER + "This is a header" + bcolors.ENDC)
# print(bcolors.OKBLUE + "This is a blue text" + bcolors.ENDC)
# print(bcolors.OKCYAN + "This is a cyan text" + bcolors.ENDC)
# print(bcolors.OKGREEN + "This is a green text" + bcolors.ENDC)
# print(bcolors.WARNING + "This is a warning text" + bcolors.ENDC)
# print(bcolors.FAIL + "This is an error text" + bcolors.ENDC)
# print(bcolors.BOLD + "This is a bold text" + bcolors.ENDC)
# print(bcolors.UNDERLINE + "This is an underlined text" + bcolors.ENDC)


def usb_port():
    print("Hardware_tool begin")

    # 舵机串口
    if os.path.exists(servo_usb_path):
        print(bcolors.OKGREEN + f"{servo_usb_path} 舵机出串口设备存在" + bcolors.ENDC)
    else:
        print(bcolors.FAIL + f"{servo_usb_path} 舵机出串口设备不存在" + bcolors.ENDC)


    # RS485 USB
    if os.path.exists(claw_usb_path):
        print(bcolors.OKGREEN + f"{claw_usb_path} 手抓串口设备存在" + bcolors.ENDC)
    else:
        print(bcolors.FAIL + f"{claw_usb_path} 手抓串口设备不存在" + bcolors.ENDC)



    result,canBus = ruierman.canbus_open()
    if(result == True):
        print(bcolors.OKGREEN + "瑞尔曼 canbus open success" + bcolors.ENDC)
    else:
        print(bcolors.FAIL + "瑞尔曼canbus open fail" + bcolors.ENDC)
    time.sleep(2)
    canBus.close_canbus()


def imu_software():
    # 定义要运行的命令
    command = "/home/lab/mtmanager/linux-x64/bin/mtmanager"  # 以 ls -l 命令为例

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)


def imu_test():
    # 定义要运行的命令
    command = "sudo /home/lab//kuavo/build/lib/xsens_ros_mti_driver/imu_test"  # 以 ls -l 命令为例

    # 使用 subprocess.run() 运行命令
    subprocess.run(command, shell=True)



if __name__ == '__main__':
    """ 
    # 获取所有连接的串口设备
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # print(port)
        print("设备: {}".format(port.device))
        print("描述: {}".format(port.description))
        print("hwid: {}".format(port.hwid))
        print("------------------------------")
"""

    # 提示用户选择
    print("请选择一个选项(q退出)：")
    print("1. 硬件连接检查")
    print("2. 打开imu上位机软件(接屏幕)")
    print("3. 测试imu")
    print("4. 瑞尔曼电机设置ID")
    print("5. 瑞尔曼电机设置零点")
    print("6. 瑞尔曼电机测试")
    print("7. 测试舵机")
    print("8. 测试手爪")

    # 获取用户输入的选项
    option = input("请输入选项编号：")

    # 如果用户选择退出，则退出循环
    if option == "q":
        print("已退出")
        exit()

    # 根据用户选择执行相应的操作
    if option == "1":
        print(bcolors.HEADER + "###开始，硬件连接检查###" + bcolors.ENDC)
        usb_port()
        print(bcolors.HEADER + "###结束，硬件连接检查###" + bcolors.ENDC)
    elif option == "2":
        print(bcolors.HEADER + "###正在打开imu上位机软件###" + bcolors.ENDC)
        imu_software()
        print(bcolors.HEADER + "###已打开imu上位机软件###" + bcolors.ENDC)
    elif option == "3":
        print(bcolors.HEADER + "###开始，测试imu###" + bcolors.ENDC)
        imu_test()
        print(bcolors.HEADER + "###结束，测试imu###" + bcolors.ENDC)
    elif option == "4":
        print(bcolors.HEADER + "###开始，设置电机ID###" + bcolors.ENDC)
        rui_id = input("请输入电机ID(左1右2): ")
        rui_id = rui_id.split(" ")

        if rui_id[0] == '1' or rui_id[0] == '2':
            pass
        else:
            print("无效的选项编号，请重新输入")
            exit()

        if len(rui_id) == 1:
            rui_id[0] = int(rui_id[0])
            rui_id.append(3)
        elif len(rui_id) == 2:
            if rui_id[0] == '1' or rui_id[0] == '2':
                pass
            else:
                print("无效的选项编号，请重新输入")
                exit()
            rui_id[0] = int(rui_id[0])
            rui_id[1] = int(rui_id[1])
        else:
            print("无效的选项编号，请重新输入")
            exit()

        new_id,old_id = rui_id
        print("新电机ID：" + str(new_id),"原电机ID：" + str(old_id), "修改ID后需重新上电")
        ruierman.ruierman_setId(new_id,old_id)
        print(bcolors.HEADER + "###结束，设置电机ID。请拔插电机电源重新上电###" + bcolors.ENDC)
    elif option == "5":
        print(bcolors.HEADER + "###开始，设置电机零点###" + bcolors.ENDC)
        ruierman.ruierman_setZero()
        print(bcolors.HEADER + "###结束，设置电机零点###" + bcolors.ENDC)
    elif option == "6":
        print(bcolors.HEADER + "###开始，电机测试###" + bcolors.ENDC)
        ruierman.ruierman_mov()
        print(bcolors.HEADER + "###结束，电机测试###" + bcolors.ENDC)
    elif option == "7":
        print(bcolors.HEADER + "###开始，测试舵机###" + bcolors.ENDC)
        dynamixel_servo.dxl_test()
        print(bcolors.HEADER + "###结束，测试舵机###" + bcolors.ENDC)
    elif option == "8":
        print(bcolors.HEADER + "###开始，测试手爪###" + bcolors.ENDC)
        claw_rs485.claw_test("/dev/claw_serial", 115200)
        print(bcolors.HEADER + "###结束，测试手爪###" + bcolors.ENDC)
    else:
        print("无效的选项编号，请重新输入")


