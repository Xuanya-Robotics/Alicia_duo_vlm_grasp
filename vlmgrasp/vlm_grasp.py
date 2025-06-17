#!/usr/bin/env python3
import signal
import rospy
import os
import sys
import numpy as np
import cv2
import dashscope
import pyaudio
from dashscope.audio.asr import *
from speech_recog import Callback
# import speech_recog
from object_detection import ObjectDetection
from tts import TextToSpeech
robot_path = os.path.expanduser('~/alicia_ws/src/alicia_duo_moveit/scripts')
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'  # Adjust this path if necessary
import time
sys.path.append(robot_path)
sys.path.append(ros_path)

from tf.transformations import quaternion_from_euler
from moveit_control import MoveItRobotController
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from camera_trans import ObjectPoseTransformer
from geometry_msgs.msg import Pose
from tf_pub import ObjecttfPublisher
sys.path.remove(ros_path)  # Remove ROS path after importing cv_bridge
bridge = CvBridge()  # Initialize CvBridge here

import pyaudio

class AudioPlayer:

    def __init__(self):
        self.mic = None
        self.stream = None
        
        channels = 1  # mono channel
        dtype = 'int16'  # data type
        block_size = 3200  # number of frames per buffer
        self.init_dashscope_api_key()  # Initialize DashScope API key
        self.callback = Callback(self.mic, self.stream)
        self.init_audio()  # Initialize audio stream
    
        # self.speech_callback = Callback()
    def init_audio(self):
        sample_rate = 16000  # sampling rate (Hz)
        format_pcm = 'pcm'  # the format of the audio data
        self.recognition = Recognition(
            model='paraformer-realtime-v2',
            # 'paraformer-realtime-v1'、'paraformer-realtime-8k-v1'
            format=format_pcm,
            # 'pcm'、'wav'、'opus'、'speex'、'aac'、'amr', you can check the supported formats in the document
            sample_rate=sample_rate,
            # support 8000, 16000
            semantic_punctuation_enabled=False,
            callback=self.callback)
        
        """Initialize audio stream"""
        try:
            self.mic = pyaudio.PyAudio()
            self.stream = self.mic.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=3200
            )
            print("Audio stream initialized successfully")
        except Exception as e:
            print(f"Error initializing audio: {e}")
    def init_dashscope_api_key(self):
        """
            Set your DashScope API-key. More information:
            https://github.com/aliyun/alibabacloud-bailian-speech-demo/blob/master/PREREQUISITES.md
        """

        if 'DASHSCOPE_API_KEY' in os.environ:
            dashscope.api_key = os.environ[
                'DASHSCOPE_API_KEY']  # load API-key from environment variable DASHSCOPE_API_KEY
        else:
            dashscope.api_key = 'sk-82d96b55ac664519b5d64c7da3ca8e60'  # set API-key manually

    def signal_handler(self, sig, frame):
        print('Ctrl+C pressed, stop translation ...')
        # Stop translation
        self.recognition.stop()
        print('Translation stopped.')
        print(
            '[Metric] requestId: {}, first package delay ms: {}, last package delay ms: {}'
            .format(
                self.recognition.get_last_request_id(),
                self.recognition.get_first_package_delay(),
                self.recognition.get_last_package_delay(),
            ))
        # Forcefully exit the program
        # Clean up audio resources
        self.cleanup()
        sys.exit(0)


    def asr(self, execution_time=3.0):
        # Ensure audio stream is available
        if not self.stream or self.stream.is_stopped():
            print("Reinitializing audio stream...")
            self.init_audio()
        # Start translation
        self.recognition.start()

        signal.signal(signal.SIGINT, self.signal_handler)
        print("Press 'Ctrl+C' to stop recording and translation...")
        # Create a keyboard listener until "Ctrl+C" is pressed
        # only loop for a limited time
        start_time = time.time()
        while time.time() - start_time < execution_time:
            if self.stream and not self.stream.is_stopped():  # Check if stream is active
                try:
                    data = self.stream.read(3200, exception_on_overflow=False)
                    self.recognition.send_audio_frame(data)
                except Exception as e:
                    print(f"Error reading audio: {e}")
                    break
            else:
                break
        
        try:
            self.recognition.stop()
            self.cleanup()
            print("Recognition stopped successfully")
        except Exception as e:
            print(f"Error stopping recognition: {e}")


    def cleanup(self):
        """Clean up audio resources"""
        # stream = self.stream
        # mic = self.mic
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
                self.stream = None
            if self.mic:
                self.mic.terminate()
                self.mic = None
        except Exception as e:
            print(f"Error cleaning up audio: {e}")

class vlm_grasp:

    def __init__(self):
        # initialize a ros node to detect the wake_word topic
        # rospy.init_node('vlm_grasp_node', anonymous=False)
        self.grasping = False
        self.releasing = False
        self.object_detection = False
        # subscribe to the wake_word topic
        self.wake_word_sub = rospy.Subscriber('wake_word', Bool, self.wake_word_callback)
        # subscribe the image topic
        self.obj_trans = ObjectPoseTransformer()
        self.detector = ObjectDetection()
        self.controller = MoveItRobotController()
        # joint state [-1.5370486974716187, -0.08436894416809082, 0.16873788833618164, -0.007669903803616762, -0.754718542098999, -0.016873788088560104]
        self.latest_object_pose = None
        self.place_pose = [0.914, 0.362, -0.075, 0.008, -0.879, -0.032]
        self.tts = TextToSpeech()

        self.speech_recog = AudioPlayer()


        # self.speech_callback = Callback()
        # self.tts.synthesize_and_play("Hello, I am ready to grasp the object.")
        # Subscribe to the speech command topic
        # self.speech_sub = rospy.Subscriber('speech_command', String, self.speech_callback_v)

    def wake_word_callback(self, msg):
        if msg.data:
            # Do the vlm detection and grasping
            self.object_detection = True

    def speech_callback_v(self, msg):
        # Process the speech command
        command = msg.data
        if command:
            self.speech_callback.latest_command = command
            rospy.loginfo(f"Received command: {command}")
        
    def detect_object(self):
        bridge = CvBridge()
        # rospy.init_node('vlm_grasp', anonymous=False)
        # 订阅 RealSense 相机的相关话题depth_camera_info_topic
        color_topic = "/camera/color/image_raw"  # 彩色图像话题
        # depth_topic = "/camera/aligned_depth_to_color/image_raw"  # 深度图像话题
        depth_topic = "/camera/depth/image_raw"  # 深度图像话题
        
        color_camera_info_topic = "/camera/color/camera_info"  # 彩色相机信息话题
        depth_camera_info_topic = "/camera/depth/camera_info"  # 深度相机信息话题
        # depth_camera_info_topic = "/camera/aligned_depth_to_color/camera_info"  # 深度相机信息话题
        # 将 ROS 消息转换为 OpenCV 格式
        camera_info = rospy.wait_for_message(color_camera_info_topic, CameraInfo, timeout=5)
        # while True:

        color_msg = rospy.wait_for_message(color_topic, Image)
        depth_msg = rospy.wait_for_message(depth_topic, Image)
            # 归一化并预处理图像
        rgb = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")  # 转换彩色图像
        depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float32)  # 转换深度图像

        print("rgb.shape:", rgb.shape)
        print("depth.shape:", depth.shape)
            
        if rgb is not None:
        #     # Perform object detection
            center_coordinates = self.detector.detect_green_cube(rgb)
            print("Center Coordinates:", center_coordinates)
            # tts play the text "I have detected len(center_coordinates) objects"

            self.tts.synthesize_and_play(f"I have detected {len(center_coordinates)} objects")

            # # tts play the object lable text
            for i in range(len(center_coordinates)):
                self.tts.synthesize_and_play(f"Object {i+1} is a {center_coordinates[i]['label']}")

        # #     # Get 3D position from detected 2D position and depth
            camera_pose = self.obj_trans.pixel_to_camera_pose(center_coordinates, depth, camera_info)
            # print("camera_pose:", camera_pose)
        # #     # Visualize results
            # self.detector.visualize_results(rgb, center_coordinates)
            # print("camera_pose:", camera_pose)
            return camera_pose
        
        return None



    def run(self):
        """
        Main execution loop with language-guided grasping
        """
        rate = rospy.Rate(2)  # 2 Hz loop rate
        rospy.loginfo("Waiting for wake word...")
        
        try:
            self.speech_recog.asr(execution_time=3.0)  # Start speech recognition
            # rospy.sleep(10.0)
            while not rospy.is_shutdown():
                self.object_detection = self.speech_recog.callback.wake_up
                # print("object_detection:", self.object_detection)
                # Check if wake word was detected
                # if True:
                if self.object_detection:
                    # self.speech_recog.asr(execution_time=6.0)
                    # rospy.sleep(10.0)
                    rospy.loginfo("Wake word detected - starting object detection")
                    # return
                    self.tts.synthesize_and_play("I heard you. Looking for objects now.")
                    
                    # # Perform object detection
                    self.latest_object_pose = self.detect_object()
                    # # # print("latest_object_pose:", self.latest_object_pose)
                    if not self.latest_object_pose or len(self.latest_object_pose) == 0:
                        rospy.logwarn("No objects detected")
                        # self.tts.synthesize_and_play("I couldn't find any objects to grasp.")
                    else:
                        rospy.loginfo(f"Detected {len(self.latest_object_pose)} objects")
                        self.tts.synthesize_and_play("Please tell me which objects to grasp.")
                        print("===================================")
                        print("Please tell me which objects to grasp.")
                        print("===================================")



                    for i in range(len(self.latest_object_pose)):
                    #     # Filter objects based on speech command
                        print(self.latest_object_pose, "self.latest_object_pose")
                        self.speech_recog.asr(execution_time=6.0)
                        time.sleep(4.0)

                    # #     # [{'label': 'Green Cube', 'camera_pose': array([0.1502891 , 0.18920869, 0.619     , 1.        ]), 'world_pose': array([ 0.32800852,  0.08553766, -0.00470618])}] self.latest_object_pose
                    # # if True:
                    # #     self.latest_object_pose = [{
                    # #         'label': 'Green Cube',
                    # #         'camera_pose': np.array([0.1502891, 0.18920869, 0.619, 1.0]),
                    # #         'world_pose': np.array([0.32800852, 0.08553766, -0.00470618])
                    # #     }]
                        objects_to_grasp = self.speech_recog.callback.object_to_grasp(self.latest_object_pose)
                    #     # rospy.loginfo(f"Objects to grasp based on command: {objects_to_grasp}")
                        print("objects_to_grasp:", objects_to_grasp)
                    #     # time.sleep(5)
                        if not objects_to_grasp:
                            print("No objects to grasp based on command")
                    #         # self.tts.synthesize_and_play(f"I didn't understand which objects to grasp.")
                        else:
                    #         # Report what will be grasped
                            object_names = [obj['label'] for obj in objects_to_grasp]
                            self.tts.synthesize_and_play(f"I'll grasp the{', '.join(object_names)}")
                            print("object_names", object_names)
                            # Process each selected object in order
                            for obj_data in objects_to_grasp:
                                print("obj_data:", obj_data)
                                label = obj_data['label']
                                world_pose = obj_data['world_pose']
                                
                                rospy.loginfo(f"Grasping {label} at world pose: {world_pose}")
                                # # Grasp this specific object
                                self.grasp(world_pose, label)
                                self.tts.synthesize_and_play(f"I have grasped the {label}object.")

                    
                    # # Reset detection flag to wait for next wake word trigger
                    self.speech_recog.callback.wake_up = False
                    # self.object_detection = False
                    self.grasp_command = ""  # Reset command
                    # # reset self.speech_callback.latest_command = command
                    self.speech_recog.callback.latest_command = ""  # Reset command
                    rospy.loginfo("Task complete. Waiting for next wake word...")
                    self.tts.synthesize_and_play("Task complete. I'm ready for your next command.")
                    # return
                
                # rate.sleep()
        
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")




    def grasp(self, world_pose, label):
        """Grasp the detected object using MoveIt controller
        
        Args:
            world_pose: 3D position [x, y, z] in world frame
            label: Object label for logging
        """
        try:
            rospy.loginfo(f"Moving to grasp {label} at position: {world_pose}")
            
                # First, move to a pre-grasp position slightly above the object
                # pre_grasp_pose = world_pose.copy()
                # pre_grasp_pose[2] += 0.1  # 10 cm above the object
            pre_grasp_pose = Pose()
            pre_grasp_pose.position.x = world_pose[0]
            pre_grasp_pose.position.y = world_pose[1]
            pre_grasp_pose.position.z = world_pose[2] + 0.05
            q = quaternion_from_euler(0, np.pi-0.175, 0)
            # q = quaternion_from_euler(np.pi, np.pi/2, 0)
            # pre_grasp_pose.orientation.x = 0
            # pre_grasp_pose.orientation.y = 0.997
            # pre_grasp_pose.orientation.z = 0
            # pre_grasp_pose.orientation.w = 0.083
            pre_grasp_pose.orientation.x = q[0]
            pre_grasp_pose.orientation.y = q[1]
            pre_grasp_pose.orientation.z = q[2]
            pre_grasp_pose.orientation.w = q[3]
                # Open the gripper
            self.controller.gripper_control(0.0)
                # Move to pre-grasp position
            # tf_publisher = ObjecttfPublisher()
            # tf_publisher.publish_tf_for_duration(pre_grasp_pose, duration=120.0)
            
            self.controller.move_to_pose(pre_grasp_pose)
                
            # # Move down to grasp position
            target_pose = pre_grasp_pose
    
            target_pose.position.z -= 0.05  # Move down to grasp position
            # target_pose.orientation.x = 0
            # target_pose.orientation.y = 0.997
            # target_pose.orientation.z = 0
            # target_pose.orientation.w = 0.083
            self.controller.move_to_pose(target_pose)
                
            #     # Close the gripper
            self.controller.gripper_control(2.0)
                
            # # place the object at a specified location
            self.controller.move_to_joint_state(self.place_pose)
            self.controller.gripper_control(0.0)
                
        except Exception as e:
            rospy.logerr(f"Error while grasping: {e}")


if __name__ == '__main__':
    vlm_grasp = vlm_grasp()
    # wait for object_detection to be true
    vlm_grasp.run()
        # Call the detect_object method to start the detection process
        # 这里可以添加一个定时器，定时调用检测函数
        # 例如：rospy.Timer(rospy.Duration(1.0), self.detect_object)
        # 但是目前我们直接在run函数中调用detect_object
        # 这样可以避免频繁调用detect_object函数
        # 也可以在run函数中添加一个定时器来控制检测频率
        # 例如：rospy.Timer(rospy.Duration(1.0), self.detect_object)
    # vlm_grasp.detect_object()