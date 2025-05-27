#!/usr/bin/env python3
import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from tf.transformations import quaternion_matrix
import os
import tf2_ros
import geometry_msgs.msg
import tf_conversions


class ObjectPoseTransformer:
    def __init__(self):
        # 加载标定变换矩阵
        self.object_detected = False  # Initialize to False
        self.transform_matrix = self.load_transform_from_yaml()
        self.latest_pose = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # 10Hz by default
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_timer_callback)
        
        # 订阅检测到的物体位置
        # rospy.Subscriber("/detected_object_position", Point, self.callback)
        rospy.loginfo("ObjectPoseTransformer initialized. Waiting for detected object position...")

    def publish_timer_callback(self, event):
        """Timer callback to continuously publish the object TF"""
        # Only publish if we have a valid detection
        if self.object_detected and self.latest_pose is not None:
            self.publish_object_tf(self.latest_pose)
        # else:
        #     rospy.loginfo_throttle(10.0, "No object detected yet, waiting for detection...")

    def load_transform_from_yaml(self):
        """从 YAML 文件加载标定变换矩阵"""
        yaml_path = os.path.expanduser("~/.ros/easy_handeye/orbbec_handeyecalibration_eye_on_base.yaml")

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                trans = [
                    data['transformation']['x'],
                    data['transformation']['y'],
                    data['transformation']['z']
                ]
                quat = [
                    data['transformation']['qx'],
                    data['transformation']['qy'],
                    data['transformation']['qz'],
                    data['transformation']['qw']
                ]
                rospy.loginfo("Loaded calibration transformation from YAML.")
                return self._create_transform(trans, quat)
        except FileNotFoundError:
            rospy.logerr(f"YAML file not found at {yaml_path}. Please check the path.")
        except KeyError as e:
            rospy.logerr(f"Missing key in YAML file: {e}")
        except Exception as e:
            rospy.logerr(f"Failed to load transformation from YAML: {e}")
        rospy.signal_shutdown("Failed to load calibration data.")
        return np.identity(4)

    def _create_transform(self, trans, rot):
        """创建 4x4 齐次变换矩阵"""
        mat = quaternion_matrix(rot)
        mat[0:3, 3] = trans
        return mat

    def publish_object_tf(self, obj_pos):
        """发布物体的 TF 坐标"""
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"  # 或 "base_link" 作为基坐标系
        t.child_frame_id = "detected_object"

        t.transform.translation.x = obj_pos[0]
        t.transform.translation.y = obj_pos[1]
        t.transform.translation.z = obj_pos[2]

        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        # Add a log message but throttle it to avoid flooding the console
        # if self.object_detected:
        #     rospy.loginfo_throttle(5.0, "Publishing detected object TF at: x=%.3f y=%.3f z=%.3f", *obj_pos)
        # else:
        #     rospy.loginfo_throttle(5.0, "Publishing default object TF (no detection)")


    def world_pose(self, camera_pose):
        """将 3D 点从相机光学坐标系转换到机械臂基坐标系"""
        try:
            # camera_pose = np.array([camera_pose.x, camera_pose.y, camera_pose.z, 1.0])  # 齐次坐标
            # camera_pose = np.array([camera_pose.x, camera_pose.y, camera_pose.z])  # 齐次坐标
            # camera pose should be in form of [x, y, z, 1]  Need to be confirm???
            # 1. 物体在相机光学帧中的坐标（齐次坐标）
            obj_optical = camera_pose

            # 2. 光学帧到相机本体帧的变换矩阵
            R_optical_to_link = np.array([
                [0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]
            ])  # 旋转矩阵 + 无平移

            # # 3. 将物体从光学帧转换到相机本体帧
            # obj_link = np.dot(R_optical_to_link, obj_optical)
            obj_link = obj_optical.copy()
            # 4. 从相机本体帧转换到机械臂基坐标系
            obj_robot = np.dot(self.transform_matrix, obj_link)

            # 更新并记录结果
            self.latest_pose = obj_robot[:3]
            # self.publish_object_tf(self.latest_pose)
            # self.object_detected = True


            # 限制日志输出频率
            rospy.loginfo_throttle(1.0, "Detected Object in Robot Frame: x=%.3f y=%.3f z=%.3f", *obj_robot[:3])
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")
        return self.latest_pose[:3]  # 返回 x, y, z 坐标





    def pixel_to_camera_pose(self, center_coordinates, depth_image, camera_info, cube_size=0.05):
        """
        Convert pixel coordinates to camera frame coordinates using depth
        Args:
            center_coordinates: List of dictionaries with 'label' and 'center' [x,y] pixel coordinates
            depth_image: Depth image from the camera
            camera_info: Camera calibration information
            cube_size: Actual size of the cube in meters (e.g., 5cm)
        Returns:
            List of 3D positions in camera frame
        """
        # Get camera intrinsics
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        
        camera_poses = []
        
        for obj in center_coordinates:
            if 'center' not in obj:
                rospy.logwarn(f"Object missing 'center' key: {obj}")
                continue
                
            # Get the center pixel coordinates
            x, y = obj['center']
            x = int(x)
            y = int(y)
            
            # Make sure coordinates are within image bounds
            if x < 0 or y < 0 or x >= depth_image.shape[1] or y >= depth_image.shape[0]:
                rospy.logwarn(f"Object {obj['label']} at ({x},{y}) is outside depth image boundaries ({depth_image.shape[1]}x{depth_image.shape[0]})")
                continue
            
            # Try different approaches to get valid depth
            depth_methods = [
                # Method 1: Small window average (original method)
                lambda: self._get_depth_from_window(depth_image, x, y, 5),
                
                # # Method 2: Larger window average
                # lambda: self._get_depth_from_window(depth_image, x, y, 11),
                
                # # Method 3: Use the bounding box if available
                # lambda: self._get_depth_from_bbox(depth_image, obj) if 'bbox' in obj else None,
                
                # # Method 4: Estimate from object size if depth still invalid
                # lambda: self._estimate_depth_from_size(obj, fx, cube_size) if 'bbox' in obj else None
            ]
            
            # Try each method until we get a valid depth
            depth_median = None
            for method in depth_methods:
                depth_result = method()
                if depth_result is not None:
                    depth_median = depth_result
                    break
                    
            # If still no valid depth, skip this object
            if depth_median is None:
                rospy.logwarn(f"No valid depth could be determined for object {obj['label']} at ({x},{y})")
                continue
                
            # Convert from pixel to camera coordinates using the pinhole camera model
            X = (x - cx) * depth_median / fx
            Y = (y - cy) * depth_median / fy
            Z = depth_median
            
            # Create homogeneous coordinates
            camera_pose = np.array([X, Y, Z, 1.0])
            
            rospy.loginfo(f"Object '{obj['label']}' at ({x},{y}): Depth={depth_median:.4f}m, Camera pose: [{X:.4f}, {Y:.4f}, {Z:.4f}]")
            
            # Store the camera frame position with the object label
            camera_poses.append({
                'label': obj['label'],
                'camera_pose': camera_pose,
                'world_pose': self.world_pose(camera_pose)
            })
        
        return camera_poses

    def _get_depth_from_window(self, depth_image, x, y, window_size):
        """Get median depth from a window around the pixel."""
        x_start = max(0, x - window_size//2)
        y_start = max(0, y - window_size//2)
        x_end = min(depth_image.shape[1], x + window_size//2 + 1)
        y_end = min(depth_image.shape[0], y + window_size//2 + 1)
        
        depth_window = depth_image[y_start:y_end, x_start:x_end]
        valid_depths = depth_window[depth_window > 0]
        
        if valid_depths.size > 0:
            return np.median(valid_depths) / 1000.0
        return None

    def _get_depth_from_bbox(self, depth_image, obj):
        """Get median depth from the entire bounding box."""
        if 'bbox' not in obj:
            return None
            
        x1, y1, x2, y2 = obj['bbox']
        # Ensure bbox coordinates are within image bounds
        x1 = max(0, int(x1))
        y1 = max(0, int(y1))
        x2 = min(depth_image.shape[1]-1, int(x2))
        y2 = min(depth_image.shape[0]-1, int(y2))
        
        if x1 >= x2 or y1 >= y2:
            return None
            
        depth_region = depth_image[y1:y2, x1:x2]
        valid_depths = depth_region[depth_region > 0]
        
        if valid_depths.size > 0:
            # Convert from mm to meters if needed
            if np.median(valid_depths) > 1000:
                return np.median(valid_depths) / 1000.0
            else:
                return np.median(valid_depths)
        return None

    def _estimate_depth_from_size(self, obj, fx, cube_size):
        """Estimate depth from apparent size and known physical size."""
        if 'bbox' not in obj:
            return None
            
        x1, y1, x2, y2 = obj['bbox']
        pixel_width = abs(x2 - x1)
        
        # Using the pinhole camera model: Z = (f * real_size) / pixel_size
        # This assumes the object is facing the camera
        estimated_depth = (fx * cube_size) / pixel_width
        
        # Sanity check - depth shouldn't be too close or too far
        if 0.1 <= estimated_depth <= 2.0:  # Between 10cm and 2m
            rospy.loginfo(f"Estimated depth from object size: {estimated_depth:.4f}m")
            return estimated_depth
        
        return None


    # def pixel_to_camera_pose(self, center_coordinates, depth_image, camera_info, cube_size=0.05):
    #     """
    #     Convert pixel coordinates to camera frame coordinates using depth
    #     Args:
    #         center_coordinates: List of dictionaries with 'label' and 'center' [x,y] pixel coordinates
    #         depth_image: Depth image from the camera
    #         camera_info: Camera calibration information
    #         cube_size: Actual size of the cube in meters (e.g., 5cm)
    #     Returns:
    #         List of 3D positions in camera frame
    #     """
    #     # Get camera intrinsics
    #     fx = camera_info.K[0]
    #     fy = camera_info.K[4]
    #     cx = camera_info.K[2]
    #     cy = camera_info.K[5]
        
    #     camera_poses = []
        
    #     for obj in center_coordinates:
    #         if 'center' not in obj:
    #             continue
                
    #         # Get the center pixel coordinates
    #         x, y = obj['center']
    #         x = int(x)
    #         y = int(y)
            
    #         # Make sure coordinates are within image bounds
    #         if x < 0 or y < 0 or x >= depth_image.shape[1] or y >= depth_image.shape[0]:
    #             continue
                
    #         # Get the depth at this pixel (use a small window average for stability)
    #         window_size = 5
    #         x_start = max(0, x - window_size//2)
    #         y_start = max(0, y - window_size//2)
    #         x_end = min(depth_image.shape[1], x + window_size//2 + 1)
    #         y_end = min(depth_image.shape[0], y + window_size//2 + 1)
            
    #         depth_window = depth_image[y_start:y_end, x_start:x_end]
    #         # Filter out zero or invalid depth values
    #         valid_depths = depth_window[depth_window > 0]
            
    #         if valid_depths.size == 0:
    #             rospy.logwarn(f"No valid depth data for object {obj['label']} at pixel ({x},{y})")
    #             continue
                
    #         # Calculate median depth in meters (assuming depth is in mm)
    #         depth_median = np.median(valid_depths) / 1000.0
            
    #         # Convert from pixel to camera coordinates using the pinhole camera model
    #         X = (x - cx) * depth_median / fx
    #         Y = (y - cy) * depth_median / fy
    #         Z = depth_median
            
    #         # Optional: Refine Z using the known cube size
    #         if 'Green Cube' in obj['label']:
    #             # If we know the dimensions of the green cube, we could refine the estimate
    #             # by calculating the expected pixel size at that depth
    #             pass
            
    #         # Create homogeneous coordinates
    #         camera_pose = np.array([X, Y, Z, 1.0])
            
    #         # Store the camera frame position with the object label
    #         camera_poses.append({
    #             'label': obj['label'],
    #             'camera_pose': camera_pose,
    #             'world_pose': self.world_pose(camera_pose)
    #         })
        
    #     return camera_poses
    