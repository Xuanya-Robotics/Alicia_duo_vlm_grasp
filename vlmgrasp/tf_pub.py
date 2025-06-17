import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

class ObjecttfPublisher:
    def __init__(self):
        # Add a TF broadcaster to the class initialization
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # Other initialization code...
    
    def publish_object_tf(self, world_pose, label="object"):
        """
        Publish a TF frame for the detected object
        
        Args:
            world_pose: [x,y,z] position in world/base frame
            label: Object label for naming the TF frame
        """
        # Create a transform stamped message
        transform = TransformStamped()
        
        # Set header information
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "base_link"  # Parent frame
        
        # Create a unique frame ID based on object label
        safe_label = label.replace(' ', '_').lower()
        transform.child_frame_id = 'test_pose'
        
        # Set the translation
        # transform.transform.translation.y = world_pose[1]
        # transform.transform.translation.z = world_pose[2]
        
        transform.transform.translation.x = world_pose.position.x
        transform.transform.translation.y = world_pose.position.y
        transform.transform.translation.z = world_pose.position.z
        
        # Set the rotation - use orientation instead of rotation
        transform.transform.rotation.x = world_pose.orientation.x
        transform.transform.rotation.y = world_pose.orientation.y
        transform.transform.rotation.z = world_pose.orientation.z
        transform.transform.rotation.w = world_pose.orientation.w
        # quat = quaternion_from_euler(0, 0, 0)
        # transform.transform.rotation.x = quat[0]
        # transform.transform.rotation.y = quat[1]
        # transform.transform.rotation.z = quat[2]
        # transform.transform.rotation.w = quat[3]
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        # rospy.loginfo(f"Published TF for {label} at position: [{world_pose[0]:.3f}, {world_pose[1]:.3f}, {world_pose[2]:.3f}]")

    
    def publish_tf_for_duration(self, world_poses, duration=60.0):

        # Publish the TF for a specified duration
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = (current_time - start_time).to_sec()
            if elapsed_time > duration:
                break
            
            # Publish the TF
            self.publish_object_tf(world_poses)
            
            rate.sleep()
        rospy.loginfo(f"TF published for {duration} seconds.")
