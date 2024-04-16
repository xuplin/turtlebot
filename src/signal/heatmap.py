#!/usr/bin/env python3

import rospy
import tf

from time import sleep
from drawnow import drawnow
from matplotlib import pyplot as plt

from Config.colorbar import *
from Node import DataSaver, Painter
from Node.Painter import colormap
from Sensors.simulation.lib import device ## simulation
# from Sensors.ms2721b.lib import device   ## device
# from Sensors.angle.lib import device   ## angle device

from Utils import timeit, root_path
from Utils.Alert import PLAY_mission_completed
from Utils.Map2d import signal_map
from Utils.Statistical import min_error as function

import matplotlib.pyplot as plt
import numpy
from matplotlib.colors import LinearSegmentedColormap
from seaborn import heatmap as hmap
from Utils.mode import find_mode

colors = [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0), (1, 0, 0)]
colormap = LinearSegmentedColormap.from_list('cmap_name', colors, N=1000)

# list of data
x, y, data = [], [], []
# data_all = []
# old_map2d = []

if __name__ == '__main__':
    # initialize node
    rospy.init_node('tf_listener')
    # print in console that the node is running
    rospy.loginfo('started listener node !')
    # create tf listener
    listener = tf.TransformListener()
    # set the node to run 1 time per second (1 hz)
    rate = rospy.Rate(1.0)
    # rate = rospy.Rate(0.1)

    plt.ion()
    plt.figure(figsize=(5, 5), dpi=100)

    # 建立儲存資料夾
    path = root_path()
    # i=0

    # 計算測量時間、初始化儀控
    with timeit(), device() as sensor:  # init sensor
        try:
            # loop forever until roscore or this node is down
            while not rospy.is_shutdown():
                try:
                    # listen to transform
                    (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    # print the transform
                    rospy.loginfo('---------')
                    rospy.loginfo('Translation: ' + str(trans))
                    rospy.loginfo('Rotation: ' + str(rot))
                    power = sensor.read_power()
                    rospy.loginfo('Power: ' + str(power))
                    # rospy.loginfo('Angle: ' + str(power))
                    bot_y, bot_x = trans[1], trans[0]
                    y.append(bot_y)
                    x.append(bot_x)
                    data.append(power)
                    # data_all.append(power)
                    # print(f'({x[-1]:.3f}, {y[-1]:.3f}): {data[-1]:.3f}')


                    # if i!=0 and len(map2d)!=0:
                    #     old_map2d = map2d
                    # i+=1
                    # print('i:', i)

                    # 建立熱圖用2d陣列
                    map2d = signal_map.map2darray(y, x, data, 0.15, error_function=function) # robot_width_value = 0.3
                    # print('data:', data, type(data))
                    # print('len(map2d:)', len(map2d))
                    
                    
                    # print('******', map2d, '********', len(map2d), '************', map2d[len(map2d)-1])
                    # if len(old_map2d)!=0:
                    #     # print('old_map2d', old_map2d)
                    #     # print('map2d', map2d)
                    #     print('data_all:', data_all)


                        # if len(old_map2d[len(old_map2d)-1]) != len(map2d[len(map2d)-1]):      # 利用最後一個的長度來判斷是不是增加了格子
                        #     print('the square add!!!!!!!!!!!!!!!!!!!!!!!!!!')
                        #     data = find_mode(data_all)    # 取眾數
                        #     data = data * len(y)
                        #     print('data mode:', data)
                        #     map2d = signal_map.map2darray(y, x, data, 0.15, error_function=function)
                        #     data_all = []    # 清空data_all
                        #     print('?????????????????????????????????????????')




                    # Painter.heatmap(path, map2d)
                    plt.clf()

                    sizeX, sizeY = map2d.shape
                    font_size = min(sizeX, sizeY)
                    ax = hmap(map2d,
                            annot=False, annot_kws={"fontsize": 15},  # 格子中的字
                            cmap=colormap, vmin=-80, vmax=-20,  # colorbar 設定
                            # cmap=colormap, vmin=-30, vmax=30,  # colorbar 設定
                            square=True,
                            cbar_kws={
                                'label': 'Power'
                            })
                    # ax.figure.axes[-1].yaxis.label.set_size(font_size)
                    # colorBar = ax.collections[0].colorbar
                    # colorBar.ax.tick_params(labelsize=font_size)
                    ax.invert_yaxis()

                    # plt.show(block=False)
                    plt.pause(0.005)
                    # plt.close("all")

                    # sleep to control the node frequency
                    rate.sleep()

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        except KeyboardInterrupt:
            # 儲存檔案
            DataSaver.map(path, map2d)  # 2d陣列
            DataSaver.map_original_data(path, y, x, data)  # 原始x, y, data資料
            rospy.loginfo("save y, x, data")
            plt.savefig(path + "/heatmap.jpg", dpi=500)


        finally:
            DataSaver.map(path, map2d)  # 2d陣列
            DataSaver.map_original_data(path, y, x, data)  # 原始x, y, data資料
            rospy.loginfo("save y, x, data")
            plt.savefig(path + "/heatmap.jpg", dpi=500)
        

        plt.ioff()