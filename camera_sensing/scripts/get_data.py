#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
import numpy as np

class Depth_Camera():

    def __init__(self):
        self.pipeline = rs.pipeline() # Frame을 받을 pipeline 생성
        self.pc = rs.pointcloud() # Point cloud 생성을 위한 인스턴스 생성
        self.config = rs.config()
        self.align = None
        self.align_to = None
        self.x=None
        
    


        # 카메라에 대한 정보 가져오기
        context = rs.context()
        connect_device = None
        if context.devices[0].get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device = context.devices[0].get_info(rs.camera_info.serial_number)
        print(" > Serial number : {}".format(connect_device))

        # config 인스턴스에 depth image와 color image resolution 설정 (D435) (https://www.intelrealsense.com/)
        self.config.enable_device(connect_device)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)    # Depth stream configuration: 1280x720 resolution, 90 frames per second
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)   # Color stream configuration: 1920x1080 resolution, BGR8 format, 30 frames per second
        





    def __del__(self):
        print("Collecting process is done.\n")


    def get_depth_data(self):
        print('Collecting depth information...')
        # 카메라의 frame이 넘어올 수 있는 길
        try:
            self.pipeline.start(self.config)
        except:
            print("There is no signal sended from depth camera.")
            print("Check connection status of camera.")
            return
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # frame 받아오기
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_info = depth_frame.as_depth_frame()

        x, y = 400, 120
        # object_distance = round((depth_info.get_distance(x, y) * 100), 2)
        object_distance = round(depth_info.get_distance(x, y), 2)
        return object_distance




    def execute(self):
        print('Collecting depth information...')
        # 카메라의 frame이 넘어올 수 있는 길
        try:
            self.pipeline.start(self.config)
        except:
            print("There is no signal sended from depth camera.")
            print("Check connection status of camera.")
            return
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        try:
            while True:

                # frame 받아오기
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                depth_info = depth_frame.as_depth_frame()

                x, y = 400, 120
                object_distance = round((depth_info.get_distance(x, y) * 100), 2)
                print("Depth : ", object_distance, "cm")
                self.x=object_distance
                
                color_image = np.asanyarray(color_frame.get_data())
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                color_image = cv2.circle(color_image, (x, y), 2, (0, 0, 255), -1)
                cv2.imshow('RealSense', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipeline.stop()
            print("┌──────────────────────────────────────┐")
            print('│ Collecting of depth info is stopped. │')
            print("└──────────────────────────────────────┘")



    def cloud_mapping(self):  

        # color_frame을 point_cloud에 mapping
        # pc.calculate를 이용해 point cloud에서 point를 얻음
        self.pc.map_to(color_frame)
        points = self.pc.calculate(depth_frame)

        # color_image 사용하기 위해 points와 color_image를 ndarray로 변환
        v = np.array(points.get_vertices()) # get_vertices() 각 점의 좌표들을 리턴
        c = color_image.reshape(-1, 3)
        # v,c 이용하여 point cloud 생성

        print(v,c)
        self.pipeline.stop()



if __name__ == "__main__":
    depth_camera = Depth_Camera()
    depth_camera.execute()

