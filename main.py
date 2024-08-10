import utime
import sensor, image, time, pyb
from machine import I2C,Pin #从 machine 模块导入 I2C、 Pin 子模块
from my_ssd1306 import SSD1306_I2C #从 ssd1306 模块中导入 SSD1306_I2C 子模块
from pyb import UART
from pyb import LED

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_whitebal(False)
sensor.set_contrast(-3)             # 设置对比度
sensor.set_brightness(3)           # 设置亮度

#sensor.set_auto_gain(False)
#init_exposure = sensor.get_exposure_us()
#print("init_exposure:",init_exposure)
#sensor.set_auto_exposure(False,exposure_us=6580) # 下午室外有阳光840
#print("exposure",sensor.get_exposure_us())

sensor.skip_frames(10)
clock = time.clock()

#sensor.set_auto_whitebal(False)
#sensor.set_contrast(-3)             # 设置对比度
#sensor.set_brightness(3)           # 设置亮度
#sensor.set_saturation(-3)          # 设置饱和度
#sensor.set_gainceiling(16)         # 设置增益上限
#sensor.set_auto_gain(False)
#sensor.set_auto_exposure(False,exposure_us=50000)


# 初始化配置
work_mode = 1           # OpenMV工作模式 1:交互模式(不可以使用WiFi测试), 2:学习模式, 3:与无人机配合的视觉工作模式
WiFi_TEST_FLAG = False  # WiFi测试模式(为True时：关闭交互模式下I2C引脚及按键的定义)
point_flight_start = 0  # 定点飞行起始点
point_flight_end = 0    # 定点飞行终止点
class Ctrl(object):
    WorkMode = 0    # OpenMV上电默认的视觉工作模式
    IsDebug = 1     # 不为调试状态时关闭某些图形显示等，有利于提高运行速度
    T_ms = 0

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 常量 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# 阈值
OUT_FIELD_THRESHOLD_RED = (20, 90, 20, 60, -10, 34)
OUT_FIELD_THRESHOLD_BLUE = (20, 90, 0, 30, -55, -17)
OUT_FIELD_THRESHOLD = [(20, 90, 20, 60, -10, 34),(20, 90, 0, 30, -55, -17)]

BLACK_THRESHOLD = [(18, 36, -28, 12, -21, 31),(9, 40, -58, 12, -29, 32),(1, 20, -38, 22, -18, 32)]     # 白天室外无阳光但光线充足  # 白天光线充足室内(3, 50, -38, 22, -28, 42)
RED_THRESHOLD = [(25, 65, 5, 50, 0, 35)]#[(10, 90, 12, 72, -8, 52),(10, 90, 22, 72, -8, 52),(10, 90, 6, 82, -18, 61),(3, 90, 72, 22, -18, 52), (3, 90, 62, 12, -18, 52), (20, 90, 12, 92, -28, 82),(3, 90, 3, 72, 5, 52),(10, 90, 12, 82, -28, 62),(10, 90, 13, 82, -28, 62)]     # 白天室内光线一般(3, 90, 72, 22, -18, 52) # 白天室外无阳光但光线充足(3, 90, 62, 12, -18, 52)  # 晚上室内(20, 90, 12, 92, -28, 82)
BLUE_THRESHOLD = [(20, 70, -20, 35, -63, 0)]#[(5, 90, -19, 42, -48, -5),(4, 40, -18, 12, -28, 2),(5, 90, -18, 92, -108, -38),(5, 45, -38, 30, -38, 4),(5, 40, -38, 32, -38, -10),(5, 55, -38, 32, -37, 1),(7, 70, -38, 22, -49, -6),(20, 70, -28, 32, -38, -8),(5, 50, -18, 32, -58, -7),(10, 90, -38, 52, -67, -18),(1, 30, -18, 32, -48, 12),(1, 36, -28, 42, -58, 2)] # (5, 90, -18, 92, -108, -38):下午设置了对比度1之前的、2/3下午有太阳调了曝光(2, 40, -28, 52, -58, -8),(1, 39, -28, 42, -48, 1)
#--------------------------------------- 以下为图像格式参数 ----------------------------------------#
IMG_WIDTH = sensor.width()      # 当前格式的图像宽度,横向x
IMG_HEIGHT = sensor.height()    # 当前格式的图像高度,纵向y
# 当前格式的图像参数：宽，高，中心x，y，中心x轴线，y轴线
IMG_PARAM = {'width':IMG_WIDTH, 'height':IMG_HEIGHT, 'center_x':int(IMG_WIDTH/2), 'center_y':int(IMG_HEIGHT/2),
             'center_line_x':(0,int(IMG_HEIGHT/2),IMG_WIDTH,int(IMG_HEIGHT/2)),
             'center_line_y':(int(IMG_WIDTH/2),0,int(IMG_WIDTH/2),IMG_HEIGHT)}
# 参数名字，便于后续便利修改
IMG_PARAM_NAME = IMG_PARAM.keys()

# 检测ROI比例(百分比%)
DETECT_ROI_RATIO = 20
# 检测ROI的中点在x轴上的位置（从左到右为0到100，单位为百分比%）
DETECT_ROI_X_RATIO = 45
# 检测ROI的中点在y轴上的位置（从上到下为0到100，单位为百分比%）
DETECT_ROI_Y_RATIO = 40
# 检测ROI
DETECT_ROI = [int((IMG_PARAM['center_x']+IMG_WIDTH/100*(DETECT_ROI_X_RATIO-50))-IMG_WIDTH*(DETECT_ROI_RATIO/100)*0.5),
              int((IMG_PARAM['center_y']+IMG_HEIGHT/100*(DETECT_ROI_Y_RATIO-50))-IMG_HEIGHT*(DETECT_ROI_RATIO/100)*0.5),
              int(IMG_WIDTH*(DETECT_ROI_RATIO/100))+1+1,
              int(IMG_HEIGHT*(DETECT_ROI_RATIO/100))+1+1
             ]
                # +1是为了修正int()转化带来的偏差。如int(12.0)其结果为11，故需要通过+1进行修正。后续同理
                # 再+1是通过观察发现了一个像素点偏差，+1修正一下，具体原因暂不明。后续同理
# 检测ROI中心(cx,cy)
DETECT_ROI_CENTER = [int(DETECT_ROI[0]+0.5*DETECT_ROI[2]),
                       int(DETECT_ROI[1]+0.5*DETECT_ROI[3])
                    ]

# 黑色圆检测ROI比例(百分比%)
CIRCLE_DETECT_ROI_RATIO = 20
# 黑色圆检测ROI的中点在x轴上的位置（从左到右为0到100，单位为百分比%）
CIRCLE_DETECT_ROI_X_RATIO = 50
# 黑色圆检测中ROI的中点在y轴上的位置（从上到下为0到100，单位为百分比%）
CIRCLE_DETECT_ROI_Y_RATIO = 40
# 黑色圆检测ROI (x,y,w,h)
CIRCLE_DETECT_ROI = [int((IMG_PARAM['center_x']+IMG_WIDTH/100*(CIRCLE_DETECT_ROI_X_RATIO-50))-IMG_WIDTH*(CIRCLE_DETECT_ROI_RATIO/100)*0.5),
                     int((IMG_PARAM['center_y']+IMG_HEIGHT/100*(CIRCLE_DETECT_ROI_Y_RATIO-50))-IMG_HEIGHT*(CIRCLE_DETECT_ROI_RATIO/100)*0.5),
                     int(IMG_WIDTH*(CIRCLE_DETECT_ROI_RATIO/100))+1+1,
                     int(IMG_HEIGHT*(CIRCLE_DETECT_ROI_RATIO/100))+1+1
                    ]
# 黑色圆检测ROI中心(cx,cy)
CIRCLE_DETECT_ROI_CENTER = [int(CIRCLE_DETECT_ROI[0]+0.5*CIRCLE_DETECT_ROI[2]),
                            int(CIRCLE_DETECT_ROI[1]+0.5*CIRCLE_DETECT_ROI[3])
                           ]

# 黑色圆检测自动曝光关闭标志(True为关闭自动曝光)
AUTO_EXPOSURE_CLOSE_FLAG_CIRCLE_DETECT = False
# 黑色圆检测手动曝光时间设置(关闭自动曝光后有效)  [白天有太阳:100000]
AUTO_EXPOSURE_TIME_CIRCLE_DETECT = 120000
# 黑色A检测自动增益关闭标志(True为关闭自动增益)
# 同理
AUTO_GAIN_CLOSE_FLAG_CIRCLE_DETECT = False
#--------------------------------------- 以下学习模式相关参数 ---------------------------------------#
MACHINE_LEARNING_LOADING_TIME_S = 5     # 学习模式加载时间(s)[没有按键打断的情况下]
MACHINE_LEARNING_NAP_TIME_S = 5         # 学习模式切换学习类型的间隔时间(颜色→形状)
MACHINE_LEARNING_FUNCTION_TOTALITY = 3  # 学习模式的环节总数【更新学习模式环节时需要更改】
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (C) OpenMV LSN 2023.07.19 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#


#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ 全局变量 $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
execute_once_flags = {'find_black_circle_sensor_set_flag':True,'find_blue_triangle_sensor_set_flag':False,
                      'find_blue_circle_sensor_set_flag':False,'find_blue_rectangle_sensor_set_flag':False}
execute_once_flag_names = execute_once_flags.keys()
#--------------------------------------- 以下学习模式相关参数 ---------------------------------------#
machine_learning_loading_flag = True    # 学习模式加载标志(有按键按下会置False，导致无法进入学习模式)
machine_learning_num = 1    # 学习模式阶段标志。1:颜色识别阶段，2:形状识别阶段。由定时器间隔一定学习时间进行切换
color_find_lock_code = 0    # 学习到的颜色编码
shape_find_lock_code = 0    # 学习到的形状编码
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ (C) OpenMV LSN 2023.07.04 $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#


#******************************************* 串口通信 *********************************************#
uart = UART(3,500000)#初始化串口 波特率 500000
#串口接收飞控缓存数据

#类的实例化
Ctr=Ctrl()

class Receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0
R=Receive()

def UartSendData(Data):
    uart.write(Data)

#串口数据解析
def ReceiveAnl(data_buf,num):
    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过
    if data_buf[4]==0x06:
        #设置模块工作模式
        Ctr.WorkMode = data_buf[5]

#串口通信协议接收
def ReceivePrepare(data):
    if R.state==0:
        if data == 0xAA:#帧头
            R.uart_buf.append(data)
            R.state = 1
        else:
            R.state = 0
    elif R.state==1:
        if data == 0xAF:
            R.uart_buf.append(data)
            R.state = 2
        else:
            R.state = 0
    elif R.state==2:
        if data == 0x05:
            R.uart_buf.append(data)
            R.state = 3
        else:
            R.state = 0
    elif R.state==3:
        if data == 0x01:#功能字
            R.state = 4
            R.uart_buf.append(data)
        else:
            R.state = 0
    elif R.state==4:
        if data == 0x06:#数据个数
            R.state = 5
            R.uart_buf.append(data)
            R._data_len = data
        else:
            R.state = 0
    elif R.state==5:    # 工作模式新增时需要在这里增加
        if data==0 or data==1 or data==2 or data==3 or data==4 or data==5 or data==6 or data==7 or data==8 or data==9 or data==10 or data==11 or data==12 or data==13:
            R.uart_buf.append(data)
            R.state = 6
        else:
            R.state = 0
    elif R.state==6:
        R.state = 0
        R.uart_buf.append(data)
        ReceiveAnl(R.uart_buf,7)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state = 0

#读取串口缓存
def UartReadBuffer():
    i = 0
    Buffer_size = uart.any()
    while i<Buffer_size:
        ReceivePrepare(uart.readchar())
        i = i + 1

#标志位数据打包
def My_Pack(flag,pos_x,pos_y,start_pos,end_pos):
    if flag == 0:
        print("flag:",flag, "No any target! pos_x:",pos_x,' pos_y:',pos_y,'start_pos',start_pos,'end_pos',end_pos)
    elif flag == 1:
        print("flag:",flag, "Find target! pos_x:",pos_x,' pos_y:',pos_y,'start_pos',start_pos,'end_pos',end_pos)
    elif flag == 2:
        print("flag:",flag, "Target in! pos_x:",pos_x,' pos_y:',pos_y,'start_pos',start_pos,'end_pos',end_pos)
    elif flag == 0xFF:
        print("flag:",flag, "Work Mode 0! pos_x:",pos_x,' pos_y:',pos_y,'start_pos',start_pos,'end_pos',end_pos)
    #elif (flag == 5):
        #print("flag:",flag, "Black circle!Fork in! pos_x",pos_x,'pos_y',pos_y)

    data_package = bytearray([0xAA,0x29,0x05,0x44,0x00,flag,pos_x>>8,pos_x,pos_y>>8,pos_y,start_pos,end_pos,0x00])

    data_package[4] = 7 # 有效数据个数插入至数据包
    # 下面生成和校验数据并插入数据包
    data_sum = 0
    for data in data_package[0:-1]:
    # 将和校验位/字节前的数据进行相加
        data_sum += data
    data_package[-1] = data_sum

    return data_package
#************************************* (C) COPYRIGHT 2019 ANO *************************************#

#******************************************* 函数 *************************************************#
def sensor_param_updata():
    # 感光元件重置后进行相关图像格式参数的更新

    global IMG_WIDTH, IMG_HEIGHT
    # 更新当前格式的图像参数：宽(横向x)，高(纵向y)
    IMG_WIDTH = sensor.width()
    IMG_HEIGHT = sensor.height()

    # 更新当前格式的图像参数：宽，高，中心x，y，中心x轴线，y轴线
    IMG_PARAM['width']  = IMG_WIDTH
    IMG_PARAM['height'] = IMG_HEIGHT
    IMG_PARAM['center_x'] = int(IMG_WIDTH/2)
    IMG_PARAM['center_y'] = int(IMG_HEIGHT/2)
    IMG_PARAM['center_line_x'] = (0,int(IMG_HEIGHT/2),IMG_WIDTH,int(IMG_HEIGHT/2))
    IMG_PARAM['center_line_y'] = (int(IMG_WIDTH/2),0,int(IMG_WIDTH/2),IMG_HEIGHT)

    # 黑色圆检测ROI参数更新
    CIRCLE_DETECT_ROI[0] = int((IMG_PARAM['center_x']+IMG_WIDTH/100*(CIRCLE_DETECT_ROI_X_RATIO-50))-IMG_WIDTH*(CIRCLE_DETECT_ROI_RATIO/100)*0.5)
    CIRCLE_DETECT_ROI[1] = int((IMG_PARAM['center_y']+IMG_HEIGHT/100*(CIRCLE_DETECT_ROI_Y_RATIO-50))-IMG_HEIGHT*(CIRCLE_DETECT_ROI_RATIO/100)*0.5)
    CIRCLE_DETECT_ROI[2] = int(IMG_WIDTH*(CIRCLE_DETECT_ROI_RATIO/100))+1+1
    CIRCLE_DETECT_ROI[3] = int(IMG_HEIGHT*(CIRCLE_DETECT_ROI_RATIO/100))+1+1

    CIRCLE_DETECT_ROI_CENTER[0] = int(CIRCLE_DETECT_ROI[0]+0.5*CIRCLE_DETECT_ROI[2])
    CIRCLE_DETECT_ROI_CENTER[1] = int(CIRCLE_DETECT_ROI[1]+0.5*CIRCLE_DETECT_ROI[3])

def execute_once_flags_renew():
    # “执行一次”标志位全部更新赋值为True。服务于工作模式的随机切换
    global execute_once_flag
    for execute_once_flag_name in execute_once_flag_names:
        execute_once_flags[execute_once_flag_name] = True

def shape_judge_out_field(my_blob):
    # 场地外学习模式使用的形状判断。三角形返回1，圆形返回2，正方形返回3，其他形状返回None

    # 色块参数，用于判断形状
    my_blob_density = my_blob.density()
    my_blob_roundness = my_blob.roundness()
    my_blob_compactness = my_blob.compactness()
    my_blob_solidity = my_blob.solidity()
    my_blob_convexity = my_blob.convexity()
    print("density",my_blob_density)
    print("roundness",my_blob_roundness)
    print("compactness",my_blob_compactness)
    print("solidity",my_blob_solidity)
    print("convexity",my_blob_convexity)

    if   (0.45<my_blob_density and my_blob_density<=0.6)\
     and (0.7<my_blob_roundness and my_blob_roundness<=1)\
     and (0.3<my_blob_compactness and my_blob_compactness<=0.8)\
     and (0.5<my_blob_solidity and my_blob_solidity<=0.65)\
     and (0.5<my_blob_convexity and my_blob_convexity<=1):
        # 如果是三角形
        return 1

    elif (0.7<my_blob_density and my_blob_density<=0.85)\
     and (0.7<my_blob_roundness and my_blob_roundness<=1)\
     and (0.5<my_blob_compactness and my_blob_compactness<=1)\
     and (0.7<my_blob_solidity and my_blob_solidity<=0.9)\
     and (0.9<my_blob_convexity and my_blob_convexity<=1):
        # 如果是圆形
        return 2

    elif (0.45<my_blob_density and my_blob_density<=1)\
     and (0.6<my_blob_roundness and my_blob_roundness<=1)\
     and (0.4<my_blob_compactness and my_blob_compactness<=0.8)\
     and (0.8<my_blob_solidity and my_blob_solidity<=1)\
     and (0.7<my_blob_convexity and my_blob_convexity<=1):
        # 如果是正方形
        return 3


def shape_judge(my_blob):
    # 场地内形状判断。三角形返回1，圆形返回2，正方形返回3，其他形状返回None

    # 色块参数，用于判断形状
    my_blob_density = my_blob.density()
    my_blob_roundness = my_blob.roundness()
    my_blob_compactness = my_blob.compactness()
    my_blob_solidity = my_blob.solidity()
    my_blob_convexity = my_blob.convexity()
    #print(red_max_blob_convexity)

    if   (0.35<my_blob_density and my_blob_density<=0.6)\
     and (0.5<my_blob_roundness and my_blob_roundness<=1)\
     and (0<my_blob_compactness and my_blob_compactness<=1)\
     and (0.4<my_blob_solidity and my_blob_solidity<=0.65)\
     and (0.6<my_blob_convexity and my_blob_convexity<=1):
        # 如果是三角形
        return 1

    elif (0.6<my_blob_density and my_blob_density<=0.8)\
     and (0.6<my_blob_roundness and my_blob_roundness<=1)\
     and (0.25<my_blob_compactness and my_blob_compactness<=0.6)\
     and (0.7<my_blob_solidity and my_blob_solidity<=0.85)\
     and (0.7<my_blob_convexity and my_blob_convexity<=1):
        # 如果是圆形
        return 2

    elif (0.4<my_blob_density and my_blob_density<=1)\
     and (0.6<my_blob_roundness and my_blob_roundness<=1)\
     and (0<my_blob_compactness and my_blob_compactness<=0.7)\
     and (0.8<my_blob_solidity and my_blob_solidity<=1)\
     and (0.5<my_blob_convexity and my_blob_convexity<=0.9):
         #如果是正方形
        return 3
def my_find_triangle(blobs):
    triangle_blobs = []
    if blobs:
        # 有色块
        for each_blob in blobs:
            if shape_judge(each_blob)==1:
            # 该色块是三角形色块
                triangle_blobs.append(each_blob)
            else:
            # 该色块非三角形色块
                pass
        if len(triangle_blobs):
            # 色块中有三角形
            return triangle_blobs
        else:
            # 色块中没有三角形
            return None
    else:
        # 无色块
        return None

def my_find_circle(blobs):
    circle_blobs = []
    if blobs:
        # 有色块
        for each_blob in blobs:
            if shape_judge(each_blob)==2:
            # 该色块是圆形色块
                circle_blobs.append(each_blob)
            else:
            # 该色块非圆形色块
                pass
        if len(circle_blobs):
        # 色块中有圆形
            return circle_blobs
        else:
        # 色块中没有圆形
            return None
    else:
        # 无色块
        return None

def my_find_rectangle(blobs):
    rectangle_blobs = []
    if blobs:
        # 有色块
        for each_blob in blobs:
            if shape_judge(each_blob)==3:
            # 该色块是正方形色块
                rectangle_blobs.append(each_blob)
            else:
            # 该色块非正方形色块
                pass
        if len(rectangle_blobs):
        # 色块中有正方形
            return rectangle_blobs
        else:
        # 色块中没有正方形
            return None
    else:
        # 无色块
        return None



def LED_white():
    # 开白灯
    LED(1).on()
    LED(2).on()
    LED(3).on()

def LED_red():
    # 开红灯
    LED(1).on()
    LED(2).off()
    LED(3).off()

def LED_green():
    # 开绿灯
    LED(1).off()
    LED(2).on()
    LED(3).off()

def LED_blue():
    # 开蓝灯
    LED(1).off()
    LED(2).off()
    LED(3).on()

def LED_yellow():
    # 开黄灯
    LED(1).on()
    LED(2).on()
    LED(3).off()

def LED_pink():
    # 开粉灯
    LED(1).on()
    LED(2).off()
    LED(3).on()

def LED_pool_blue():
    # 开浅蓝灯
    LED(1).off()
    LED(2).on()
    LED(3).on()

def LED_close():
    # 关灯
    LED(1).off()
    LED(2).off()
    LED(3).off()
#******************************** (C) OpenMV LSN 2023.07.08 ***************************************#

############################################ 工作模式 ##############################################
# 工作模式1、2：红色三角形
def find_red_triangle(img):
    red_blobs = img.find_blobs(RED_THRESHOLD,merge=False,pixels_threshold=300)
    red_triangle_blobs = my_find_triangle(red_blobs)
    if red_triangle_blobs:
        # 如果找到红色三角形色块
        red_max_blob = max(red_triangle_blobs, key = lambda b: b.pixels())
        if  (DETECT_ROI[0]<red_max_blob[5]) and (red_max_blob[5]<DETECT_ROI[0]+DETECT_ROI[2]) \
            and (DETECT_ROI[1]<red_max_blob[6]) and (red_max_blob[6]<DETECT_ROI[1]+DETECT_ROI[3]):
            # Target in DETECT_ROI
            # 0x02为与主控通信的标志位，表明目标进入ROI。
            # 后续参数则是以ROI中心为xOy坐标原点，发送目标的中心坐标给主控。后续同理
            # “图像”、“视野”在本代码中若无特殊情况，视为同一含义
            UartSendData(My_Pack(0x02,red_max_blob[5]-DETECT_ROI_CENTER[0],-(red_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_close()
        else:
            # Target not in DETECT_ROI
            # 0x01表明视野中看到了目标
            UartSendData(My_Pack(0x01,red_max_blob[5]-DETECT_ROI_CENTER[0],-(red_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_white()

        # 画出红色三角形的色块中心、边框并在其上用字符串标识
        img.draw_cross(red_max_blob[5],red_max_blob[6])
        img.draw_rectangle(red_max_blob.rect(), color=(255,0,0) ,thickness=2)
        img.draw_string(red_max_blob[0],(red_max_blob[1]-15),"Triangle",color=(255,0,0),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
        # 画出图像中心、ROI及其中心
        img.draw_line(IMG_PARAM['center_line_x'],color=(0,255,0))
        img.draw_line(IMG_PARAM['center_line_y'],color=(0,255,0))
        img.draw_rectangle(DETECT_ROI, color=(255,0,0))
        img.draw_cross(DETECT_ROI_CENTER[0],DETECT_ROI_CENTER[1],color=(255,0,0))
    else:
        # 无红色三角形
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_red()

# 工作模式3、4：蓝色三角形
def find_blue_triangle(img):
    if execute_once_flags['find_blue_triangle_sensor_set_flag']:
        execute_once_flags_renew()
        execute_once_flags['find_blue_triangle_sensor_set_flag'] = False
        #sensor.set_auto_whitebal(False)
        #sensor.set_contrast(-3)             # 设置对比度
        sensor.set_brightness(3)           # 设置亮度
        sensor.set_saturation(-3)          # 设置饱和度
        #sensor.set_gainceiling(16)         # 设置增益上限
        sensor.set_auto_gain(False)
        sensor.set_auto_exposure(False,exposure_us=121300)
        sensor.skip_frames(10)


    blue_blobs = img.find_blobs(BLUE_THRESHOLD,merge=False,pixels_threshold=300)
    blue_triangle_blobs = my_find_triangle(blue_blobs)
    if blue_triangle_blobs:
        # 如果找到蓝色三角形色块
        blue_max_blob = max(blue_triangle_blobs, key = lambda b: b.pixels())
        if  (DETECT_ROI[0]<blue_max_blob[5]) and (blue_max_blob[5]<DETECT_ROI[0]+DETECT_ROI[2]) \
            and (DETECT_ROI[1]<blue_max_blob[6]) and (blue_max_blob[6]<DETECT_ROI[1]+DETECT_ROI[3]):
            # Target in DETECT_ROI
            # 0x02为与主控通信的标志位，表明目标进入ROI。
            # 后续参数则是以ROI中心为xOy坐标原点，发送目标的中心坐标给主控。后续同理
            # “图像”、“视野”在本代码中若无特殊情况，视为同一含义
            UartSendData(My_Pack(0x02,blue_max_blob[5]-DETECT_ROI_CENTER[0],-(blue_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_close()
        else:
            # Target not in DETECT_ROI
            # 0x01表明视野中看到了目标
            UartSendData(My_Pack(0x01,blue_max_blob[5]-DETECT_ROI_CENTER[0],-(blue_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_white()
        # 画出蓝色三角形的色块中心、边框并在其上用字符串标识
        img.draw_cross(blue_max_blob[5],blue_max_blob[6])
        img.draw_rectangle(blue_max_blob.rect(), color=(0,0,255) ,thickness=2)
        img.draw_string(blue_max_blob[0],(blue_max_blob[1]-15),"Triangle",color=(0,0,255),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
        # 画出图像中心、ROI及其中心
        img.draw_line(IMG_PARAM['center_line_x'],color=(0,255,0))
        img.draw_line(IMG_PARAM['center_line_y'],color=(0,255,0))
        img.draw_rectangle(DETECT_ROI, color=(255,0,0))
        img.draw_cross(DETECT_ROI_CENTER[0],DETECT_ROI_CENTER[1],color=(255,0,0))
    else:
        # 无蓝色三角形
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_blue()

# 工作模式5、6：红色圆形
def find_red_circle(img):
    red_blobs = img.find_blobs(RED_THRESHOLD,merge=False,pixels_threshold=300)
    red_circle_blobs = my_find_circle(red_blobs)
    if red_circle_blobs:
        # 如果找到红色圆形色块
        red_max_blob = max(red_circle_blobs, key = lambda b: b.pixels())
        if  (DETECT_ROI[0]<red_max_blob[5]) and (red_max_blob[5]<DETECT_ROI[0]+DETECT_ROI[2]) \
            and (DETECT_ROI[1]<red_max_blob[6]) and (red_max_blob[6]<DETECT_ROI[1]+DETECT_ROI[3]):
            # Target in DETECT_ROI
            # 0x02为与主控通信的标志位，表明目标进入ROI。
            # 后续参数则是以ROI中心为xOy坐标原点，发送目标的中心坐标给主控。后续同理
            # “图像”、“视野”在本代码中若无特殊情况，视为同一含义
            UartSendData(My_Pack(0x02,red_max_blob[5]-DETECT_ROI_CENTER[0],-(red_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_close()
        else:
            # Target not in DETECT_ROI
            # 0x01表明视野中看到了目标
            UartSendData(My_Pack(0x01,red_max_blob[5]-DETECT_ROI_CENTER[0],-(red_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_white()

        # 画出红色圆形的色块中心、边框并在其上用字符串标识
        img.draw_cross(red_max_blob[5],red_max_blob[6])
        img.draw_rectangle(red_max_blob.rect(), color=(255,0,0) ,thickness=2)
        img.draw_string(red_max_blob[0],(red_max_blob[1]-15),"Circle",color=(255,0,0),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
        # 画出图像中心、ROI及其中心
        img.draw_line(IMG_PARAM['center_line_x'],color=(0,255,0))
        img.draw_line(IMG_PARAM['center_line_y'],color=(0,255,0))
        img.draw_rectangle(DETECT_ROI, color=(255,0,0))
        img.draw_cross(DETECT_ROI_CENTER[0],DETECT_ROI_CENTER[1],color=(255,0,0))
    else:
        # 无红色圆形
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_red()

# 工作模式7、8：蓝色圆形
def find_blue_circle(img):
    if execute_once_flags['find_blue_circle_sensor_set_flag']:
        execute_once_flags_renew()
        execute_once_flags['find_blue_circle_sensor_set_flag'] = False
        #sensor.set_auto_whitebal(False)
        #sensor.set_contrast(-3)             # 设置对比度
        sensor.set_brightness(3)           # 设置亮度
        sensor.set_saturation(-3)          # 设置饱和度
        #sensor.set_gainceiling(16)         # 设置增益上限
        sensor.set_auto_gain(False)
        sensor.set_auto_exposure(False,exposure_us=101300)
        sensor.skip_frames(10)

    blue_blobs = img.find_blobs(BLUE_THRESHOLD,merge=False,pixels_threshold=300)
    blue_circle_blobs = my_find_circle(blue_blobs)
    if blue_circle_blobs:
        # 如果找到蓝色圆形色块
        blue_max_blob = max(blue_circle_blobs, key = lambda b: b.pixels())
        if  (DETECT_ROI[0]<blue_max_blob[5]) and (blue_max_blob[5]<DETECT_ROI[0]+DETECT_ROI[2]) \
            and (DETECT_ROI[1]<blue_max_blob[6]) and (blue_max_blob[6]<DETECT_ROI[1]+DETECT_ROI[3]):
            # Target in DETECT_ROI
            # 0x02为与主控通信的标志位，表明目标进入ROI。
            # 后续参数则是以ROI中心为xOy坐标原点，发送目标的中心坐标给主控。后续同理
            # “图像”、“视野”在本代码中若无特殊情况，视为同一含义
            UartSendData(My_Pack(0x02,blue_max_blob[5]-DETECT_ROI_CENTER[0],-(blue_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_close()
        else:
            # Target not in DETECT_ROI
            # 0x01表明视野中看到了目标
            UartSendData(My_Pack(0x01,blue_max_blob[5]-DETECT_ROI_CENTER[0],-(blue_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_white()
        # 画出蓝色圆形的色块中心、边框并在其上用字符串标识
        img.draw_cross(blue_max_blob[5],blue_max_blob[6])
        img.draw_rectangle(blue_max_blob.rect(), color=(0,0,255) ,thickness=2)
        img.draw_string(blue_max_blob[0],(blue_max_blob[1]-15),"Circle",color=(0,0,255),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
        # 画出图像中心、ROI及其中心
        img.draw_line(IMG_PARAM['center_line_x'],color=(0,255,0))
        img.draw_line(IMG_PARAM['center_line_y'],color=(0,255,0))
        img.draw_rectangle(DETECT_ROI, color=(255,0,0))
        img.draw_cross(DETECT_ROI_CENTER[0],DETECT_ROI_CENTER[1],color=(255,0,0))
    else:
        # 无蓝色圆形
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_blue()


# 工作模式9、10：红色正方形
def find_red_rectangle(img):
    red_blobs = img.find_blobs(RED_THRESHOLD,merge=False,pixels_threshold=300)
    #for red_blob in red_blobs:
        #img.draw_rectangle(red_blob.rect())

    red_rectangle_blobs = my_find_rectangle(red_blobs)
    if red_rectangle_blobs:
        # 如果找到红色正方形色块
        red_max_blob = max(red_rectangle_blobs, key = lambda b: b.pixels())
        if  (DETECT_ROI[0]<red_max_blob[5]) and (red_max_blob[5]<DETECT_ROI[0]+DETECT_ROI[2]) \
            and (DETECT_ROI[1]<red_max_blob[6]) and (red_max_blob[6]<DETECT_ROI[1]+DETECT_ROI[3]):
            # Target in DETECT_ROI
            # 0x02为与主控通信的标志位，表明目标进入ROI。
            # 后续参数则是以ROI中心为xOy坐标原点，发送目标的中心坐标给主控。后续同理
            # “图像”、“视野”在本代码中若无特殊情况，视为同一含义
            UartSendData(My_Pack(0x02,red_max_blob[5]-DETECT_ROI_CENTER[0],-(red_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_close()
        else:
            # Target not in DETECT_ROI
            # 0x01表明视野中看到了目标
            UartSendData(My_Pack(0x01,red_max_blob[5]-DETECT_ROI_CENTER[0],-(red_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_white()

        # 画出红色正方形的色块中心、边框并在其上用字符串标识
        img.draw_cross(red_max_blob[5],red_max_blob[6])
        img.draw_rectangle(red_max_blob.rect(), color=(255,0,0) ,thickness=2)
        img.draw_string(red_max_blob[0],(red_max_blob[1]-15),"Rectangle",color=(255,0,0),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
        # 画出图像中心、ROI及其中心
        img.draw_line(IMG_PARAM['center_line_x'],color=(0,255,0))
        img.draw_line(IMG_PARAM['center_line_y'],color=(0,255,0))
        img.draw_rectangle(DETECT_ROI, color=(255,0,0))
        img.draw_cross(DETECT_ROI_CENTER[0],DETECT_ROI_CENTER[1],color=(255,0,0))
    else:
        # 无红色正方形
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_red()

# 工作模式11、12：蓝色正方形
def find_blue_rectangle(img):
    if execute_once_flags['find_blue_rectangle_sensor_set_flag']:
        execute_once_flags_renew()
        execute_once_flags['find_blue_rectangle_sensor_set_flag'] = False
        #sensor.set_auto_whitebal(False)
        #sensor.set_contrast(-3)             # 设置对比度
        sensor.set_brightness(3)           # 设置亮度
        sensor.set_saturation(-3)          # 设置饱和度
        #sensor.set_gainceiling(16)         # 设置增益上限
        sensor.set_auto_gain(False)
        sensor.set_auto_exposure(False,exposure_us=101300)
        sensor.skip_frames(10)

    blue_blobs = img.find_blobs(BLUE_THRESHOLD,merge=False,pixels_threshold=300)
    blue_rectangle_blobs = my_find_rectangle(blue_blobs)
    if blue_rectangle_blobs:
        # 如果找到蓝色正方形色块
        blue_max_blob = max(blue_rectangle_blobs, key = lambda b: b.pixels())
        if  (DETECT_ROI[0]<blue_max_blob[5]) and (blue_max_blob[5]<DETECT_ROI[0]+DETECT_ROI[2]) \
            and (DETECT_ROI[1]<blue_max_blob[6]) and (blue_max_blob[6]<DETECT_ROI[1]+DETECT_ROI[3]):
            # Target in DETECT_ROI
            # 0x02为与主控通信的标志位，表明目标进入ROI。
            # 后续参数则是以ROI中心为xOy坐标原点，发送目标的中心坐标给主控。后续同理
            # “图像”、“视野”在本代码中若无特殊情况，视为同一含义
            UartSendData(My_Pack(0x02,blue_max_blob[5]-DETECT_ROI_CENTER[0],-(blue_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_close()
        else:
            # Target not in DETECT_ROI
            # 0x01表明视野中看到了目标
            UartSendData(My_Pack(0x01,blue_max_blob[5]-DETECT_ROI_CENTER[0],-(blue_max_blob[6]-DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
            LED_white()
        # 画出蓝色正方形的色块中心、边框并在其上用字符串标识
        img.draw_cross(blue_max_blob[5],blue_max_blob[6])
        img.draw_rectangle(blue_max_blob.rect(), color=(0,0,255) ,thickness=2)
        img.draw_string(blue_max_blob[0],(blue_max_blob[1]-15),"Rectangle",color=(0,0,255),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
        # 画出图像中心、ROI及其中心
        img.draw_line(IMG_PARAM['center_line_x'],color=(0,255,0))
        img.draw_line(IMG_PARAM['center_line_y'],color=(0,255,0))
        img.draw_rectangle(DETECT_ROI, color=(255,0,0))
        img.draw_cross(DETECT_ROI_CENTER[0],DETECT_ROI_CENTER[1],color=(255,0,0))
    else:
        # 无蓝色正方形
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_blue()

# 工作模式13
def find_black_circle(img):
    rect_of_circle =    {'x':0, 'y':0, 'width':0, 'height':0}

    if execute_once_flags['find_black_circle_sensor_set_flag']:
        execute_once_flags_renew()
        execute_once_flags['find_black_circle_sensor_set_flag'] = False
        sensor.reset()
        sensor.set_framesize(sensor.QQVGA)  # Set frame size to QVGA (320x240)
        sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
        sensor.set_auto_whitebal(False)
        sensor.set_contrast(-3)             # 设置对比度
        sensor.set_brightness(3)           # 设置亮度
        #sensor.set_contrast(3)              # 设置对比度
        #sensor.set_brightness(-3)           # 设置亮度
        #sensor.set_saturation(-3)           # 设置饱和度
        #sensor.set_gainceiling(16)          # 设置增益上限
        #sensor.set_auto_whitebal(False)     # 白平衡关闭以追踪颜色
        if AUTO_GAIN_CLOSE_FLAG_CIRCLE_DETECT:
            # 关闭自动增益
            sensor.set_auto_gain(False)
        if AUTO_EXPOSURE_CLOSE_FLAG_CIRCLE_DETECT:
            # 关闭自动曝光
            sensor.set_auto_exposure(False,exposure_us=AUTO_EXPOSURE_TIME_CIRCLE_DETECT)
            # 打印重设后的曝光值
            print("Black Circle Detect Exposure Time:%d us" % sensor.get_exposure_us())
        sensor.skip_frames(10)
        sensor_param_updata()               # 待传感器稳定后进行参数获取与更新

    circles = img.find_circles(threshold = 5000, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2)
    # threshold可以调整圆检测的“振动”，margin可在一定程度上认为是圆检测的最小下限参数参数
    if circles:
    # 如果图像中能找到圆
        max_circle = max(circles, key = lambda c: c[2])
        c_x = max_circle[0]                                     # 圆心x坐标
        c_y = max_circle[1]                                     # 圆心y坐标
        c_r = max_circle[2]                                     # 圆半径
        img.draw_circle(c_x, c_y, c_r, color = (255, 0, 0))
        rect_of_circle['x'] = c_x-c_r                           # 圆所在的正方形区域的左顶点x坐标
        rect_of_circle['y'] = c_y-c_r                           # 圆所在的正方形区域的左顶点y坐标
        rect_of_circle['width'] = 2*c_r                         # 圆所在的正方形区域的宽
        rect_of_circle['height'] = 2*c_r                        # 圆所在的正方形区域的高
        # 圆所在的正方形区域作为一个ROI
        rect_of_c_roi = (rect_of_circle['x'], rect_of_circle['y'], rect_of_circle['width'], rect_of_circle['height'])
        # 在圆所在正方形区域中寻找黑色色块
        blobs = img.find_blobs(BLACK_THRESHOLD, roi=rect_of_c_roi, x_stride=5, y_stride=5, area_threshold=20, merge=True)
        if blobs:
        # 圆中有黑色色块
            max_blob = max(blobs, key = lambda b:b.pixels())# 获得最大黑色块
            img.draw_cross(max_blob[5],max_blob[6])         # 黑色块中心
            img.draw_rectangle(max_blob.rect())             # 画的是色块的矩形，不是圆所在的正方形区域
            # 画出图像中心到圆心的连线
            img.draw_line(CIRCLE_DETECT_ROI_CENTER[0], CIRCLE_DETECT_ROI_CENTER[1], c_x, c_y, color=(255,255,0))
            if (CIRCLE_DETECT_ROI[0]<c_x) and (c_x<CIRCLE_DETECT_ROI[0]+CIRCLE_DETECT_ROI[2]) \
                and (CIRCLE_DETECT_ROI[1]<c_y) and (c_y<CIRCLE_DETECT_ROI[1]+CIRCLE_DETECT_ROI[3]):
                # Circle in CIRCLE_DETECT_ROI
                # 发送0x02给主控，表明黑色圆形进入图像CIRCLE_DETECT_ROI
                # 后续参数为：在以ROI中心为xOy坐标原点的坐标系下，黑色圆心的坐标。后续同理
                UartSendData(My_Pack\
                (0x02,c_x-CIRCLE_DETECT_ROI_CENTER[0],-(c_y-CIRCLE_DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
                LED_close() # 关灯表示黑色圆进入视野中央
            else:
                # Cirlce not in CIRCLE_DETECT_ROI
                # 发送0x01给主控，表明黑色圆形进入视野
                UartSendData(My_Pack\
                (0x01,c_x-CIRCLE_DETECT_ROI_CENTER[0],-(c_y-CIRCLE_DETECT_ROI_CENTER[1]),point_flight_start,point_flight_end))
                LED_white() # 白灯表示黑色圆进入视野
            # 画出图像中心、ROI及其中心
            img.draw_line(IMG_PARAM['center_line_x'],color=(0,0,255))
            img.draw_line(IMG_PARAM['center_line_y'],color=(0,0,255))
            img.draw_rectangle(CIRCLE_DETECT_ROI, color=(255,0,0))
            img.draw_cross(CIRCLE_DETECT_ROI_CENTER[0],CIRCLE_DETECT_ROI_CENTER[1],color=(255,0,0))
        else:
        # 圆中无黑色块
            UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
            LED_pink()
    else:
    # 如果图像中找不到圆
        UartSendData(My_Pack(0x00,0,0,point_flight_start,point_flight_end))
        LED_pink()
    # 在该模式下，上述逻辑即在没有任何识别的情况下亮蓝灯，以示该模式正在工作
################################## (C) OpenMV LSN 2023.07.08 #######################################

#************************************** 按键&交互功能定义 *******************************************#
if WiFi_TEST_FLAG == False:
    # 当不进行WiFi扩展版调试时，产生下列定义用于oled显示
    i2c = I2C(sda=Pin("P7"), scl=Pin("P8"),freq=80000)
    oled = SSD1306_I2C(128, 64, i2c, addr=0x3c)

#----------------------------------------- 按键读取 ------------------------------------------------#
    button_num = 0              # 记录按下的按键号码
    def get_buttonNum():
    # 用于主循环中读取按键
        global button_num
        temp = 0
        temp = button_num       # 读取按键值，后续用于函数返回
        button_num = 0          # 读取一次按键值后，将按键值清零(防止程序认为按键一直按下)
        return temp             # 返回本次函数执行读取到的按键值

    # pin脚对象定义，上拉输入(“pin脚”，即“引脚”)
    pin0 = pyb.Pin("P2", pyb.Pin.IN, pyb.Pin.PULL_UP)
    pin1 = pyb.Pin("P3", pyb.Pin.IN, pyb.Pin.PULL_UP)
    pin2 = pyb.Pin("P6", pyb.Pin.IN, pyb.Pin.PULL_UP)
    pin3 = pyb.Pin("P9", pyb.Pin.IN, pyb.Pin.PULL_UP)
    #pin4 = pyb.Pin("P4", pyb.Pin.IN, pyb.Pin.PULL_UP)

    def get_pinNum():
    # 获取按键按下对应的pin脚号码。有号码代表对应号码的pin脚为低电平，号码为-1代表所有按键pin脚均为高电平
        pin_Num = -1

        if pin0.value() == 0:
            pin_Num = 0
        if pin1.value() == 0:
            pin_Num = 1
        if pin2.value() == 0:
            pin_Num = 2
        if pin3.value() == 0:
            pin_Num = 3
        #if pin4.value() == 0:
            #pin_Num = 4

        return pin_Num


    now_pinNum = -1     # 记录上一次按键扫描时按键被按下的pin脚号码(有按下则有号码，无按下则为-1)
    last_pinNum = -1    # 记录这一次按键扫描时按键被按下的pin脚号码
    def button_scanLoop():
        global now_pinNum, last_pinNum,button_num
        last_pinNum = now_pinNum    # 更新上一次按键扫描时按键被按下的pin脚号码
        now_pinNum = get_pinNum()   # 获取现在按键被按下的pin脚号码

        if now_pinNum == -1:        # 现在已经松开按键,即没有pin脚处于低电平
            if last_pinNum == 0:    # 上一时刻pin0脚的按键按下
                button_num = 1      # 说明KEY1按下。
            elif last_pinNum == 1:  # 后续同上
                button_num = 2
            elif last_pinNum == 2:
                button_num = 4
            elif last_pinNum == 3:
                button_num = 5
            #elif last_pinNum == 4:
                #button_num = 6

#--------------------------------------------------------------------------------------------------#

#---------------------------------------- 显示相关变量 ---------------------------------------------#
    # HOME界面
    HOME_FUNCTION_TOTAL_TOTALITY = 3  # HOME界面功能总数【增加功能需更新】
    home_function_select_num = 0    # 当前要选中的HOME界面功能号
    # 定点飞行设置界面
    set_point_flight_num = 1        # 当前要设置的定点飞行项。1:设置Start, 2:设置End
#----------------------------------------- 显示内容 ------------------------------------------------#
    def show_home_function_select_content():
        oled.fill(0)                # 清屏
        oled.text("HOME",48,0)      # 从0行48列开始显示“HOME”
        oled.text("HN:",96,0)       # 显示当前要选中HOME界面的功能号
        oled.text(str(home_function_select_num),120,0) # 显示当前要选中HOME界面的功能号
        oled.text("1.Point",0,8)
        oled.text("2.Learning",0,16)
        oled.text("3.Enter",0,24)
        #oled.text("4.Function4",0,32)
        #oled.text("5.Function5",0,40)
        #oled.text("6.Function6",0,48)
        #oled.text("7.Function7",0,56)
        oled.show()                 # OLED显示生效

    def show_set_point_flight_content():
        oled.fill(0)
        oled.text("Set Flight Point",0,0)
        oled.text("SN:",96,56)
        oled.text(str(set_point_flight_num),120,56) # 显示当前要选中HOME界面的功能号
        oled.text("1.Start:",0,16)
        oled.text("2.End:",0,32)
        oled.text(str(point_flight_start),64,16)    # 显示起始飞行点设置
        oled.text(str(point_flight_end),64,32)      # 显示最终飞行点设置
        oled.show()

    def show_machine_learning_content():
        oled.fill(0)
        oled.text("Machine Learning",0,0)
        oled.text("developing...",16,32)
        oled.show()

    def show_enter_vision_work_content():
        oled.fill(0)
        oled.text("Enter work mode?",0,24)
        oled.show()

    def show_set_ok():
        oled.fill(0)
        oled.text("Set OK!",40,24)
        oled.show()

    def show_enter():
    # 显示进入定点飞行模式
        oled.fill(0)
        oled.text("Enter!",40,24)
        oled.show()

    #def show_enter_machine_learning():
    ## 显示进入机器学习模式
        #oled.fill(0)
        #oled.text("Machine Learning",0,0)
        #oled.text("developing...",16,32)
        #oled.show()

#--------------------------------------------------------------------------------------------------#

#----------------------------------------- 交互功能 ------------------------------------------------#
    # 显示更新相关标志及处理
        # 【增加功能需更新】0:Home界面；1：设定飞行地点
    display_renew_flags = { 'home_function0':True, 'home_function1':True,
                            'home_function2':True, 'home_function3':True }


    display_renew_flag_names=display_renew_flags.keys()
    def display_renew_flags_renew():
    # “显示更新”标志位全部更新赋值为True。服务于交互界面的随机切换显示
        for display_renew_flag_name in display_renew_flag_names:
            display_renew_flags[display_renew_flag_name] = True

    # HOME界面功能号宏定义
    HOME_FUNCTION_SELECT = 0
    SET_POINT_FLIGHT = 1
    MACHINE_LEARNING = 2
    ENTER_VISION_WORK = 3

    # Home界面及该界面相关功能标志位
    home_function_flag = 0  # 起始界面功能模式选择。0:显示Home界面
    def home_function_select(button_value):
        global home_function_select_num
        if display_renew_flags['home_function0']:
        # 功能界面显示
            display_renew_flags_renew()
            display_renew_flags['home_function0'] = False
            show_home_function_select_content()

        if button_value==1:
        # HOME界面按键1按下。将当前功能号返回，以进入相应功能
            temp = home_function_select_num
            home_function_select_num = 0
            return temp
        elif button_value==2:
        # HOME界面按键2按下,当前功能号++
            home_function_select_num += 1
            if home_function_select_num>HOME_FUNCTION_TOTAL_TOTALITY:
                home_function_select_num=0      # 功能号大于系统功能总数时归零
            show_home_function_select_content() # 显示更新
            return HOME_FUNCTION_SELECT         # 保持HOME界面功能
        else:
        # Home界面无按键或其他按键按下。保持HOME界面功能
            return HOME_FUNCTION_SELECT

    # 定点飞行设置界面功能号宏定义
    START_SET = 1   # 设置起始点功能号
    END_SET = 2     # 设置终止点功能号
    # 定点飞行设置界面
    def set_point_flight(button_value):
        global set_point_flight_num,point_flight_start,point_flight_end
        if display_renew_flags['home_function1']:
        # 功能界面显示
            display_renew_flags_renew()
            display_renew_flags['home_function1'] = False
            show_set_point_flight_content()

        if button_value==1:
        # 确认键
            set_point_flight_num += 1               # 准备进入设置下一个功能号的设置
            if set_point_flight_num>END_SET:
            # 功能号全部设置完毕时显示“Set OK!”，并将下一个功能号设置回到Start
                show_set_ok()                       # 显示“Set OK!”
                utime.sleep_ms(1000)                # 显示停留1s
                set_point_flight_num = START_SET    # 功能号设置为START_SET，即回到设置界面后从START开始设置
            show_set_point_flight_content()         # 更新显示当前设置功能号
            return SET_POINT_FLIGHT                 # 继续显示定点飞行设置界面
        elif button_value==2:
        # 参数设置++键
            if set_point_flight_num==START_SET:
            # 设置Start++
                point_flight_start+=1
                if point_flight_start>=13:
                    point_flight_start=0        # Start大于上限12时归零
                show_set_point_flight_content() # 更新显示Start
            elif set_point_flight_num==END_SET:
            # 设置End++
                point_flight_end+=1
                if point_flight_end>=13:
                    point_flight_end=0          # End大于上限12时归零
                show_set_point_flight_content() # 更新显示End
            return SET_POINT_FLIGHT             # 继续显示定点飞行设置界面
        elif button_value==5:
        # 返回键。返回HOME界面
            return HOME_FUNCTION_SELECT
        else:
        # 其他按键按下或无按键按下。继续当前定点飞行设置界面
            return SET_POINT_FLIGHT

    # 机器学习模式界面
    def machine_learning(button_value):
        if display_renew_flags['home_function2']:
        # 功能界面显示
            display_renew_flags_renew()
            display_renew_flags['home_function2'] = False
            show_machine_learning_content()
        if button_value==1:
            return MACHINE_LEARNING
        elif button_value==5:
            return HOME_FUNCTION_SELECT
        else:
            return MACHINE_LEARNING

    # 进入视觉工作状态设置界面功能号宏定义
    # 进入视觉工作状态设置界面
    def enter_vision_work(button_value):
        global work_mode
        if display_renew_flags['home_function3']:
        # 功能界面显示
            display_renew_flags_renew()
            display_renew_flags['home_function3'] = False
            show_enter_vision_work_content()
        if button_value==1:
        # 确认键
            show_enter()
            work_mode = 3
            return HOME_FUNCTION_SELECT
        elif button_value==5:
        # 返回键
            return HOME_FUNCTION_SELECT
        else:
            return ENTER_VISION_WORK
        # 其他按键按下或无按键按下。继续当前进入视觉工作状态设置界面

#--------------------------------------------------------------------------------------------------#

#******************************** (C) OpenMV LSN 2023.07.19 ***************************************#


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 学习功能 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
red_blob_record_times = 0   # 颜色学习到红色色块的次数
blue_blob_record_times = 0  # 颜色学习到蓝色色块的次数
def machine_learning_color(img):
# 颜色学习
    global red_blob_record_times,blue_blob_record_times
    blobs = img.find_blobs(OUT_FIELD_THRESHOLD, merge=False, pixels_threshold=300)
    if blobs:
    # 有红、蓝色的色块
        max_blob = max(blobs, key = lambda b: b.pixels())   # 获取初识别阶段识别到的最大色块
        color_code = max_blob.code()                        # 判断色块颜色代码
        if color_code==1:
        # 红色色块
            img.draw_rectangle(max_blob.rect(),color=(255,0,0),thickness=2)
            red_blob_record_times+=1    # 红色色块识别次数+1
        elif color_code==2:
        # 蓝色色块
            img.draw_rectangle(max_blob.rect(),color=(0,0,255),thickness=2)
            blue_blob_record_times+=1   # 蓝色色块识别次数+1
        else:
        # 颜色代码出错
            print("Color code error!")
    else:
    # 无红、蓝色的色块
        print("No blob!")
    # 将本阶段识别/捕获次数最多的色块颜色锁定为下一阶段的目标识别颜色
    if red_blob_record_times>blue_blob_record_times:
    # 红色识别/捕获次数最多，则锁定为红色(返回1)
        LED_red()
        return 1
    elif red_blob_record_times<blue_blob_record_times:
    # 蓝色识别/捕获次数最多，锁定为蓝色(返回2)
        LED_blue()
        return 2
    else:
    # 蓝色/红色识别次数相同或其他，锁定无效(返回0)
        LED_white()
        print("Color findind lock false!")
        return 0

blob_shape_record = []
def machine_learning_shape(img, color_find_lock_code):
    if color_find_lock_code==1:
    # 锁定学习红色色块的形状
        red_blobs = img.find_blobs([OUT_FIELD_THRESHOLD_RED], merge=False, pixels_threshold=300)
        if red_blobs:
            max_red_blob = max(red_blobs, key = lambda b: b.pixels())
            img.draw_rectangle(max_red_blob.rect(),color=(0,255,0),thickness=2)
            shape_code = shape_judge_out_field(max_red_blob)
            blob_shape_record.append(shape_code)    # 将识别到的形状编码记录至表中(注意没有识别到形状时记录的是"None")
            if shape_code==1:
                img.draw_string(max_red_blob[0],(max_red_blob[1]-15),"Triangle",color=(255,0,0),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
            elif shape_code==2:
                img.draw_string(max_red_blob[0],(max_red_blob[1]-15),"Circle",color=(255,0,0),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
            elif shape_code==3:
                img.draw_string(max_red_blob[0],(max_red_blob[1]-15),"Rectangle",color=(255,0,0),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
    elif color_find_lock_code==2:
    # 锁定学习蓝色色块
        blue_blobs = img.find_blobs([OUT_FIELD_THRESHOLD_BLUE], merge=False, pixels_threshold=300)
        if blue_blobs:
            max_blue_blob = max(blue_blobs, key = lambda b: b.pixels())
            img.draw_rectangle(max_blue_blob.rect(),color=(0,255,0),thickness=2)
            shape_code = shape_judge_out_field(max_blue_blob)
            blob_shape_record.append(shape_code)    # 将识别到的形状编码记录至表中(注意没有识别到形状时记录的是"None")
            if shape_code==1:
                img.draw_string(max_blue_blob[0],(max_blue_blob[1]-15),"Triangle",color=(0,0,255),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
            elif shape_code==2:
                img.draw_string(max_blue_blob[0],(max_blue_blob[1]-15),"Circle",color=(0,0,255),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
            elif shape_code==3:
                img.draw_string(max_blue_blob[0],(max_blue_blob[1]-15),"Rectangle",color=(0,0,255),scale=1.5,mono_space = False,char_rotation = 0, char_hmirror = False, char_vflip = False,string_rotation = 0, string_hmirror = False, string_vflip = False)
    else:
    # 锁定学习的颜色代码有误
        print("Color find lock code error!")


    if blob_shape_record:
    # 如果锁定识别颜色后，有发生色块形状识别(注意blob_shape_record里面可能存在None，这是因为某一瞬间没有识别出形状产生的)
        if (blob_shape_record.count(1)>blob_shape_record.count(2)) and (blob_shape_record.count(1)>blob_shape_record.count(3)):
        # 识别到最多的形状是三角形，并锁定(返回1)
            LED_pink()
            return 1
        elif (blob_shape_record.count(2)>blob_shape_record.count(1)) and (blob_shape_record.count(2)>blob_shape_record.count(3)):
        # 识别到最多的形状是圆形，并锁定(返回2)
            LED_yellow()
            return 2
        elif (blob_shape_record.count(3)>blob_shape_record.count(1)) and (blob_shape_record.count(3)>blob_shape_record.count(2)):
        # 识别到最多的形状是正方形，并锁定(返回3)
            LED_pool_blue()
            return 3
        else:
        # 形状识别锁定有误
            LED_white()
            print("Shape find lock error!")
            return 0
    else:
    # 没有发生色块形状识别
        LED_white()
        print("Shape record false!")

def machine_learning_decode(color_find_lock_code,shape_find_lock_code):
    if color_find_lock_code==1:
    # 学习到的目标是红色
        if shape_find_lock_code==1:
        # 学习到的目标是三角形
            print("Red Triangle!")
            return 1, 2
        elif shape_find_lock_code==2:
        # 学习到的目标是圆形
            print("Red Circle!")
            return 5, 6
        elif shape_find_lock_code==3:
        # 学习到的目标是正方形
            print("Red Rectangle!")
            return 9, 10
        else:
        # 没有学习到形状
            print("Red Nothing!")
            return 0, 0
    elif color_find_lock_code==2:
    # 学习到的目标是蓝色
        if shape_find_lock_code==1:
        # 学习到的目标是三角形
            print("Blue Triangle!")
            return 3, 4
        elif shape_find_lock_code==2:
        # 学习到的目标是圆形
            print("Blue Circle!")
            return 7, 8
        elif shape_find_lock_code==3:
        # 学习到的目标是正方形
            print("Blue Rectangle!")
            return 11, 12
        else:
        # 没有学习到形状
            print("Blue Nothing!")
            return 0, 0
    else:
    # 没有学习到红色或蓝色
        if shape_find_lock_code==1:
        # 学习到的目标是三角形
            print("Nothing Triangle!")
        elif shape_find_lock_code==2:
        # 学习到的目标是圆形
            print("Nothing Circle!")
        elif shape_find_lock_code==3:
        # 学习到的目标是正方形
            print("Nothing Rectangle!")
        else:
        # 没有学习到形状
            print("Nothing Nothing!")
        # 识别失败，返回(0,0)
        return 0, 0
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ (C) OpenMV LSN 2023.07.19 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

############################################# 定时器4 ###############################################
timer4 = pyb.Timer(4, freq=1000)    # 定时器溢出周期为1ms
count0 = 0
count1 = 0
count2 = 0
def timer4_interrupt(t):    # 作为定时器的回调函数这个参数t是必要的！尽管它没用
    global count0,count1,count2,work_mode,machine_learning_num
    count0 += 1
    count1 += 1
    if work_mode==2:
    # 在openmv系统工作模式2下才对学习模式切换的间隔时间进行计时
        count2 += 1

    if WiFi_TEST_FLAG==False:
    # 在不执行WiFi调试的情况下
        if count0>=20:
        # 每间20ms执行一次按键扫描
            count0 = 0
            button_scanLoop()

    if (count1>=MACHINE_LEARNING_LOADING_TIME_S*1000) and work_mode<2:
    # 上电经过加载时间后判断是否进入学习模式。已经进入学习模式及其之后的模式了就不再执行该判断语句里的逻辑
        if machine_learning_loading_flag == True:
        # 如果此期间内学习模式加载标志没有被打断，则进入学习模式
            work_mode = 2
            print("Enter machine learning mode!")

    if (count2>=MACHINE_LEARNING_NAP_TIME_S*1000) and machine_learning_num<=MACHINE_LEARNING_FUNCTION_TOTALITY:
    # 每间隔学习时间切换学习的类别(颜色→形状)[注意这里没间隔该时间machine_learning_num]
        machine_learning_num += 1
        count2 = 0

timer4.callback(timer4_interrupt)   # 每1ms执行一次定时器“中断”函数
################################## (C) OpenMV LSN 2023.07.20 #######################################


print("Machine_learning_loading...")
# 主循环
while(True): #重复执行
    #print(work_mode)
    if (work_mode==1) and (WiFi_TEST_FLAG==False):
    # 按键交互模式
        button_value = get_buttonNum()
        if button_value and machine_learning_loading_flag:
        # 如果有按键按下则将学习模式的加载标志置为无效(后续无法进入学习模式)；若学习模式的加载标志已经无效了则不进入该语句
            machine_learning_loading_flag=False # 置学习模式的加载标志为无效
            print("Machine learning loading break!")

        if home_function_flag==HOME_FUNCTION_SELECT:
        # Home界面交互
            home_function_flag = home_function_select(button_value)
        elif home_function_flag==SET_POINT_FLIGHT:
        # 进入定点飞行设置
            home_function_flag = set_point_flight(button_value)
        elif home_function_flag==MACHINE_LEARNING:
        # 进入机器学习模式
            home_function_flag = machine_learning(button_value)
        elif home_function_flag==ENTER_VISION_WORK:
        # 进入视觉工作状态
            home_function_flag = enter_vision_work(button_value)

    elif work_mode == 2:
    # 学习模式
        clock.tick()
        img = sensor.snapshot()

        if machine_learning_num==1:
        # 识别/捕获待学习目标的颜色
            color_find_lock_code = machine_learning_color(img)

        elif machine_learning_num==2:
         # 识别/捕获待学习目标的形状(利用上一模式捕获色块的色块颜色，对该种颜色的色块进行形状判断)
         # 如果上一阶段颜色学习失败，那么这一阶段将无法进行形状学习
            if color_find_lock_code:
            # 如果上一阶段成功锁定了颜色，则进一步锁定形状(需要使用锁定目标颜色的代码：1表示红色,2表示蓝色)
                shape_find_lock_code = machine_learning_shape(img, color_find_lock_code)
            else:
            # 上一阶段没有锁定目标颜色
                print("Color findind lock false!")

        elif machine_learning_num==3:
        # 颜色和形状学习完成。解码为对应形状的起始点和终止点
            point_flight_start, point_flight_end = machine_learning_decode(color_find_lock_code,shape_find_lock_code)
            work_mode = 3 # 解码完成，进入与无人机配合的视觉工作模式

    elif work_mode == 3:
    # 与无人机配合的视觉工作模式
        clock.tick()
        img = sensor.snapshot()
        UartReadBuffer()
        if Ctr.WorkMode==0:
        # 无工作，亮黄灯
            UartSendData(My_Pack(0xFF,0,0,point_flight_start,point_flight_end))
            LED_green()
        elif Ctr.WorkMode==1:
        # 寻找红色三角形1
            find_red_triangle(img)
        elif Ctr.WorkMode==2:
        # 寻找红色三角形2
            find_red_triangle(img)
        elif Ctr.WorkMode==3:
        # 寻找蓝色三角形3
            find_blue_triangle(img)
        elif Ctr.WorkMode==4:
        # 寻找蓝色三角形4
            find_blue_triangle(img)
        elif Ctr.WorkMode==5:
        # 寻找红色圆形5
            find_red_circle(img)
        elif Ctr.WorkMode==6:
        # 寻找红色圆形6
             find_red_circle(img)
        elif Ctr.WorkMode==7:
        # 寻找蓝色圆形7
            find_blue_circle(img)
        elif Ctr.WorkMode==8:
        # 寻找蓝色圆形8
             find_blue_circle(img)
        elif Ctr.WorkMode==9:
        # 寻找红色正方形9
            find_red_rectangle(img)
        elif Ctr.WorkMode==10:
        # 寻找红色正方形10
            find_red_rectangle(img)
        elif Ctr.WorkMode==11:
        # 寻找蓝色正方形11
            find_blue_rectangle(img)
        elif Ctr.WorkMode==12:
        # 寻找蓝色正方形12
            find_blue_rectangle(img)
        elif Ctr.WorkMode==13:
        # 寻找黑色圆
            find_black_circle(img)
        else:
        # 错误工作模式
            LED_pool_blue()
