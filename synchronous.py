# from dlclive import DLCLive, prograss

# Import helper functions and classes written to wrap the RealSense, OpenCV and Kabsch Calibration usage
from collections import defaultdict
import ctypes
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing

#import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures

import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import time
import os
from queue import Empty
import threading
from multiprocessing import Process,Queue,Manager,Value
import multiprocessing
from neuracle_lib.triggerBox import TriggerBox,TriggerIn,PackageSensorPara
import logging

# 配置日志系统
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')


minute_input = 40

save_path = 'D:/Video_Recording/test/2/'

date = 'lhp20240313'

stream_frame = 30

save_frame = 30.0

width = 1280
height = 720

isTriggerIn = True

COM = "COM4"

trigger_value = 1

trigger_list = []

def get_all_true(bool_list):
    for i in bool_list:
        if not i.value:
            return False
    return True

def get_max_recorded(frames_recorded_list):
    frames_values = [frame.value for frame in frames_recorded_list]
    
    max_frames = max(frames_values)
    return max_frames

def listen_for_exit(stop_signal):
    print("Press 'q' to exit")
    while True:
        time.sleep(0.5)
        if input() == 'q':
            stop_signal.value = True
            break

def send_trigger(value):
    if isTriggerIn:
        triggerin = TriggerIn(COM)
        flag = triggerin.validate_device()        
        if flag:
            triggerin.output_event_data(value)
            trigger_list.append(value)
            #print(trigger_list)
        else:
            raise Exception("Invalid Serial!")

def control(s,e1,e2,e3,e4,e5,e6,e7,e8,frames,stop_signal, recorded_done_list,first_frame_list, new_frame_list, begin_send, end_send):
    global trigger_value
    global trigger_list
    f=1
    frames=int(frames['frames'])
    trigger_flag = 2
    
    while True:
        if f>=frames:
            logging.info("Reached the end of frames.")
            break
        elif stop_signal.value:
            logging.info("Stop signal received.")
            while True:
                s.get()
                if get_all_true(recorded_done_list):
                    send_trigger(255)
                    logging.info("All recorded done, sent trigger 255.")
                    break
            break
        elif get_all_true(first_frame_list):
            send_trigger(0)
            logging.info("Send begin trigger.")
            begin_send.value = True
            for i in range(0, 8):
                first_frame_list[i].value = False
        elif get_all_true(new_frame_list):
            if end_send.value:
                send_trigger(0)
                begin_send.value = True
                end_send.value = False
                for i in range(0, 8):
                    new_frame_list[i].value = False
        if s.qsize()==8:
            if trigger_flag == 300 and begin_send.value:
                send_trigger(trigger_value)
                end_send.value = True
                begin_send.value = False
                trigger_value += 1
                trigger_flag = 1
            else:
                trigger_flag += 1
            for i in range(8):
                s.get()

            e1.set()
            e2.set()
            e3.set()
            e4.set()
            e5.set()
            e6.set()
            e7.set()
            e8.set()

            f += 1
        else:
            continue
    print(trigger_list)

def streamv1(q1,e1,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frames1 = int(frames['frames'])
    frame_stop = int(frames1)
    frame_count = 1
    new_count = 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('234222303417')
    config_1.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_1.start(config_1)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[0].value = True
            break

        frames_1 = pipeline_1.wait_for_frames()

        color_frame_1 = frames_1.get_color_frame()

        color_image_1 = np.asanyarray(color_frame_1.get_data())
        q1.put(color_image_1)
        s.put(1)

        if frame_count == 1:
            first_frame_list[0].value = True
        if new_count == 300:
            new_frame_list[0].value = True
            new_count = 1
        if not stop_signal.value:
            e1.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[0].value = frame_count
    time_end = time.time()
    print('1st-time cost', time_end - time_start, 's')
    pipeline_1.stop()

def streamv2(q2,e2,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device('239222303314')
    config_2.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_2.start(config_2)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[1].value = True
            break
        frames_2 = pipeline_2.wait_for_frames()

        color_frame_2 = frames_2.get_color_frame()

        color_image_2 = np.asanyarray(color_frame_2.get_data())
        q2.put(color_image_2)
        s.put(1)
        if frame_count == 1:
            first_frame_list[1].value = True
        if new_count == 300:
            new_frame_list[1].value = True
            new_count = 1
        if not stop_signal.value:
            e2.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[1].vaule = frame_count
    time_end = time.time()
    print('2st-time cost', time_end - time_start, 's')
    pipeline_2.stop()

def streamv3(q3,e3,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_3 = rs.pipeline()
    config_3 = rs.config()
    config_3.enable_device('235422301124')
    config_3.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_3.start(config_3)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[2].value = True
            break

        frames_3 = pipeline_3.wait_for_frames()

        color_frame_3 = frames_3.get_color_frame()

        color_image_3 = np.asanyarray(color_frame_3.get_data())
        q3.put(color_image_3)
        s.put(1)
        if frame_count == 1:
            first_frame_list[2].value = True
        if new_count == 300:
            new_frame_list[2].value = True
            new_count = 1
        if not stop_signal.value:
            e3.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[2].value = frame_count
    time_end = time.time()
    print('3st-time cost', time_end - time_start, 's')
    pipeline_3.stop()

def streamv4(q4,e4,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_4 = rs.pipeline()
    config_4 = rs.config()
    config_4.enable_device('234222300830')
    config_4.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_4.start(config_4)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[3].value = True
            break

        frames_4 = pipeline_4.wait_for_frames()

        color_frame_4 = frames_4.get_color_frame()

        color_image_4 = np.asanyarray(color_frame_4.get_data())
        q4.put(color_image_4)
        s.put(1)
        if frame_count == 1:
            first_frame_list[3].value = True
        if new_count == 300:
            new_frame_list[3].value = True
            new_count = 1
        if not stop_signal.value:
            e4.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[3].value = frame_count

    time_end = time.time()
    print('4st-time cost', time_end - time_start, 's')
    pipeline_4.stop()

def streamv5(q5,e5,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_5 = rs.pipeline()
    config_5 = rs.config()
    config_5.enable_device('239222301376')
    config_5.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_5.start(config_5)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[4].value = True
            break

        frames_5 = pipeline_5.wait_for_frames()

        color_frame_5 = frames_5.get_color_frame()

        color_image_5 = np.asanyarray(color_frame_5.get_data())
        q5.put(color_image_5)
        s.put(1)
        if frame_count == 1:
            first_frame_list[4].value = True
        if new_count == 300:
            new_frame_list[4].value = True
            new_count = 1
        if not stop_signal.value:
            e5.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[4].value = frame_count
    time_end = time.time()
    print('5st-time cost', time_end - time_start, 's')
    pipeline_5.stop()

def streamv6(q6,e6,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_6 = rs.pipeline()
    config_6 = rs.config()
    config_6.enable_device('234322306734')
    config_6.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_6.start(config_6)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[5].value = True
            break

        frames_6 = pipeline_6.wait_for_frames()

        color_frame_6 = frames_6.get_color_frame()
        color_image_6 = np.asanyarray(color_frame_6.get_data())
        q6.put(color_image_6)
        s.put(1)
        if frame_count == 1:
            first_frame_list[5].value = True
        if new_count == 300:
            new_frame_list[5].value = True
            new_count = 1
        if not stop_signal.value:
            e6.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[5].value = frame_count
    time_end = time.time()
    print('6st-time cost', time_end - time_start, 's')
    pipeline_6.stop()

def streamv7(q7,e7,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_7 = rs.pipeline()
    config_7 = rs.config()
    config_7.enable_device('239222302137')
    config_7.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_7.start(config_7)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[6].value = True
            break

        frames_7 = pipeline_7.wait_for_frames()

        color_frame_7 = frames_7.get_color_frame()

        color_image_7 = np.asanyarray(color_frame_7.get_data())
        q7.put(color_image_7)
        s.put(1)
        if frame_count == 1:
            first_frame_list[6].value = True
        if new_count == 300:
            new_frame_list[6].value = True
            new_count = 1
        if not stop_signal.value:
            e7.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[6].value = frame_count
    time_end = time.time()
    print('7st-time cost', time_end - time_start, 's')
    pipeline_7.stop()

def streamv8(q8,e8,s,frames, first_frame_list, stop_signal, frames_recorded_list, recorded_done_list, new_frame_list):
    frame_stop = int(frames['frames'])

    frame_count = 1
    new_count = 1
    pipeline_8 = rs.pipeline()
    config_8 = rs.config()
    config_8.enable_device('234322303283')
    config_8.enable_stream(rs.stream.color, width,height, rs.format.bgr8, stream_frame)
    pipeline_8.start(config_8)
    time.sleep(3)
    time_start = time.time()
    while True:
        if frame_count == frame_stop:
            break
        elif stop_signal.value and abs(frame_count - get_max_recorded(frames_recorded_list)) <= 1:
            recorded_done_list[7].value = True
            break

        frames_8 = pipeline_8.wait_for_frames()

        color_frame_8 = frames_8.get_color_frame()

        color_image_8 = np.asanyarray(color_frame_8.get_data())
        q8.put(color_image_8)
        s.put(1)
        if frame_count == 1:
            first_frame_list[7].value = True
        if new_count == 300:
            new_frame_list[7].value = True
            new_count = 1
        if not stop_signal.value:
            e8.wait(2)


        frame_count = frame_count + 1
        new_count = new_count + 1
        frames_recorded_list[7].value = frame_count
    time_end = time.time()
    print('8st-time cost', time_end - time_start, 's')
    pipeline_8.stop()


def savev1(q1,frames):
    frames1 = int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out1=cv2.VideoWriter(frames['sp'] + frames['date'] +'-'+'-' + '1.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q1.get(timeout=15)
            out1.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera1 stopping...")
            break
    print(frames1)
    print(f)
    out1.release()


def savev2(q2,frames):
    frames1=int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out2=cv2.VideoWriter(frames['sp'] + frames['date'] +'-' + '-'  + '2.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q2.get(timeout=15)
            out2.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera2 stopping...")
            break
    out2.release()




def savev3(q3,frames):
    frames1 = int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out3=cv2.VideoWriter(frames['sp'] + frames['date']+'-' +'-' + '3.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q3.get(timeout=15)
            out3.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera3 stopping...")
            break
    out3.release()



def savev4(q4,frames):
    frames1 = int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out4=cv2.VideoWriter(frames['sp'] + frames['date'] +'-'+'-' + '4.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q4.get(timeout=15) 
            out4.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera4 stopping...")
            break

    out4.release()


def savev5(q5,frames):
    frames1 = int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out5=cv2.VideoWriter(frames['sp'] + frames['date'] +'-' +'-' + '5.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q5.get(timeout=15)
            out5.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera5 stopping...")
            break
    out5.release()


def savev6(q6,frames):
    frames1 = int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out6=cv2.VideoWriter(frames['sp'] + frames['date'] +'-' +'-' + '6.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q6.get(timeout=15)
            out6.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera6 stopping...")
            break
    out6.release()


def savev7(q7,frames):
    frames1 = int(frames['frames'])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out7=cv2.VideoWriter(frames['sp'] + frames['date'] +'-' +'-' + '7.avi',fourcc, save_frame, (width,height))
    f = 1
    while frames1>f:
        try:
            frame = q7.get(timeout=15)
            out7.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera7 stopping...")
            break
    out7.release()

def savev8(q8,frames):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out8=cv2.VideoWriter(frames['sp'] + frames['date'] +'-'+'-' + '8.avi',fourcc, save_frame, (width,height))
    frames1 = int(frames['frames'])
    f = 1
    while frames1>f:
        try:
            frame = q8.get(timeout=15)
            out8.write(frame)
            f=f+1
        except Empty:
            print("No new frames received in 15 seconds, Cmaera8 stopping...")
            break
    out8.release()






if __name__ == '__main__':
    lock = threading.Lock()
    stop_signal = multiprocessing.Value('b', False)  # 按'q' 结束对应的停止信号
    exit_listener = threading.Thread(target=listen_for_exit, args=(stop_signal,))
    exit_listener.start()
    frames_recorded1 = multiprocessing.Value('i', 0)
    frames_recorded2 = multiprocessing.Value('i', 0)
    frames_recorded3 = multiprocessing.Value('i', 0)
    frames_recorded4 = multiprocessing.Value('i', 0)
    frames_recorded5 = multiprocessing.Value('i', 0)
    frames_recorded6 = multiprocessing.Value('i', 0)
    frames_recorded7 = multiprocessing.Value('i', 0)
    frames_recorded8 = multiprocessing.Value('i', 0)
    frames_recorded_list = [frames_recorded1, frames_recorded2, frames_recorded3, frames_recorded4, frames_recorded5, frames_recorded6, frames_recorded7, frames_recorded8]

    recorded_done1 = multiprocessing.Value('b', False)
    recorded_done2 = multiprocessing.Value('b', False)
    recorded_done3 = multiprocessing.Value('b', False)
    recorded_done4 = multiprocessing.Value('b', False)
    recorded_done5 = multiprocessing.Value('b', False)
    recorded_done6 = multiprocessing.Value('b', False)
    recorded_done7 = multiprocessing.Value('b', False)
    recorded_done8 = multiprocessing.Value('b', False)
    recorded_done_list = [recorded_done1, recorded_done2, recorded_done3, recorded_done4, recorded_done5, recorded_done6, recorded_done7, recorded_done8]

    first_frame1 = multiprocessing.Value('b', False)
    first_frame2 = multiprocessing.Value('b', False)
    first_frame3 = multiprocessing.Value('b', False)
    first_frame4 = multiprocessing.Value('b', False)
    first_frame5 = multiprocessing.Value('b', False)
    first_frame6 = multiprocessing.Value('b', False)
    first_frame7 = multiprocessing.Value('b', False)
    first_frame8 = multiprocessing.Value('b', False)
    first_frame_list = [first_frame1, first_frame2, first_frame3, first_frame4, first_frame5, first_frame6, first_frame7 ,first_frame8]

    new_frame1 = multiprocessing.Value('b', False)
    new_frame2 = multiprocessing.Value('b', False)
    new_frame3 = multiprocessing.Value('b', False)
    new_frame4 = multiprocessing.Value('b', False)
    new_frame5 = multiprocessing.Value('b', False)
    new_frame6 = multiprocessing.Value('b', False)
    new_frame7 = multiprocessing.Value('b', False)
    new_frame8 = multiprocessing.Value('b', False)
    new_frame_list = [new_frame1, new_frame2, new_frame3, new_frame4, new_frame5, new_frame6, new_frame7, new_frame8]

    begin_send = multiprocessing.Value('b', False)
    end_send = multiprocessing.Value('b', False)

    m = multiprocessing.Manager()
    q1 = Queue(0)
    q2 = Queue(0)
    q3 = Queue(0)
    q4 = Queue(0)
    q5 = Queue(0)
    q6 = Queue(0)
    q7 = Queue(0)
    q8 = Queue(0)
    minute = minute_input
    frames_temp=minute*30*60+1
    frames_temp=int(frames_temp)
    frames = multiprocessing.Manager().dict()

    frames['frames']=frames_temp
    frames['sp'] = save_path
    frames['date'] = date

    s = Queue(8)
    e1 = multiprocessing.Event()
    e2 = multiprocessing.Event()
    e3 = multiprocessing.Event()
    e4 = multiprocessing.Event()
    e5 = multiprocessing.Event()
    e6 = multiprocessing.Event()
    e7 = multiprocessing.Event()
    e8 = multiprocessing.Event()
    p0 = Process(target=control,args=(s,e1,e2,e3,e4,e5,e6,e7,e8,frames,stop_signal,recorded_done_list,first_frame_list,new_frame_list,begin_send,end_send,))
    p0.start()

    pst1 = Process(target=streamv1,args=(q1,e1,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst1.start()
    pst2 = Process(target=streamv2,args=(q2,e2,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst2.start()
    pst3 = Process(target=streamv3,args=(q3,e3,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst3.start()
    pst4 = Process(target=streamv4,args=(q4,e4,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst4.start()
    pst5 = Process(target=streamv5,args=(q5,e5,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst5.start()
    pst6 = Process(target=streamv6,args=(q6,e6,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst6.start()
    pst7 = Process(target=streamv7,args=(q7,e7,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst7.start()
    pst8 = Process(target=streamv8,args=(q8,e8,s,frames,first_frame_list,stop_signal,frames_recorded_list,recorded_done_list,new_frame_list,))
    pst8.start()

    psv1 = Process(target=savev1,args=(q1,frames,))
    psv1.start()
    psv2 = Process(target=savev2,args=(q2,frames,))
    psv2.start()
    psv3 = Process(target=savev3,args=(q3,frames,))
    psv3.start()
    psv4 = Process(target=savev4,args=(q4,frames,))
    psv4.start()
    psv5 = Process(target=savev5,args=(q5,frames,))
    psv5.start()
    psv6 = Process(target=savev6,args=(q6,frames,))
    psv6.start()
    psv7 = Process(target=savev7,args=(q7,frames,))
    psv7.start()
    psv8 = Process(target=savev8,args=(q8,frames,))
    psv8.start()

    p0.join()
    pst1.join()
    pst2.join()
    pst3.join()
    pst4.join()
    pst5.join()
    pst6.join()
    pst7.join()
    pst8.join()

    psv1.join()
    psv2.join()
    psv3.join()
    psv4.join()
    psv5.join()
    psv6.join()
    psv7.join()
    psv8.join()

    exit_listener.join()