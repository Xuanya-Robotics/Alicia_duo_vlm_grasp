# For prerequisites running the following sample, visit https://help.aliyun.com/zh/model-studio/getting-started/first-api-call-to-qwen
import os
import signal  # for keyboard events handling (press "Ctrl+C" to terminate recording and translation)
import sys
import rospy
import dashscope
import pyaudio
from dashscope.audio.asr import *
from std_msgs.msg import Bool, String
mic = None
stream = None

# # Set recording parameters
# sample_rate = 16000  # sampling rate (Hz)
# channels = 1  # mono channel
# dtype = 'int16'  # data type
# format_pcm = 'pcm'  # the format of the audio data
# block_size = 3200  # number of frames per buffer


def init_dashscope_api_key():
    """
        Set your DashScope API-key. More information:
        https://github.com/aliyun/alibabacloud-bailian-speech-demo/blob/master/PREREQUISITES.md
    """

    if 'DASHSCOPE_API_KEY' in os.environ:
        dashscope.api_key = os.environ[
            'DASHSCOPE_API_KEY']  # load API-key from environment variable DASHSCOPE_API_KEY
    else:
        dashscope.api_key = 'your api key'  # set API-key manually


# Real-time speech recognition callback
class Callback(RecognitionCallback):

    def __init__(self, mic, stream) -> None:
        self.mic = mic
        self.stream = stream
        # sample_rate = 16000  # sampling rate (Hz)
        # channels = 1  # mono channel
        # dtype = 'int16'  # data type
        # format_pcm = 'pcm'  # the format of the audio data
        # block_size = 3200  # number of frames per buffer

        try:
            rospy.init_node('speech_recognition_node', anonymous=True)
            print("ROS node initialized")
        except Exception as e:
            print(f"Could not initialize ROS node: {e}")
        self.wake_word_pub = rospy.Publisher('wake_word', Bool, queue_size=10)
        self.command_pub = rospy.Publisher('speech_command', String, queue_size=10)
        # subscribe to speech_command topic
        self.command_sub = rospy.Subscriber('speech_command', String, self.command_subscallback)
        # Store the latest command - no need for separate publisher
        self.latest_command = ""
        self.latest_text = ""
        self.wake_up = None

    def command_subscallback(self, msg):
        """Callback for speech_command topic"""
        self.latest_command = msg.data
        print(f"Received command from topic: {self.latest_command}")
        # You can add additional processing here if needed
        # For example, you might want to trigger some action based on the command

    def on_open(self) -> None:
        # mic = self.mic
        # stream = self.stream
        print('RecognitionCallback open.')
        # Don't create new audio stream if one is already provided
        if not self.mic or not self.stream:
            self.mic = pyaudio.PyAudio()
            self.stream = self.mic.open(format=pyaudio.paInt16,
                              channels=1,
                              rate=16000,
                              input=True)


    def on_close(self) -> None:
        print('RecognitionCallback close.')
        self.stream.stop_stream()
        self.stream.close()
        self.mic.terminate()
        self.stream = None
        self.mic = None

    def on_complete(self) -> None:
        print('RecognitionCallback completed.')  # translation completed

    def on_error(self, message) -> None:
        print('RecognitionCallback task_id: ', message.request_id)
        print('RecognitionCallback error: ', message.message)
        # Stop and close the audio stream if it is running
        # Stop and close the audio stream if it is running
        try:
            if self.stream and hasattr(self.stream, 'is_active') and self.stream.is_active():
                self.stream.stop_stream()
                self.stream.close()
        except Exception as e:
            print(f"Error closing stream: {e}")
        # if 'stream' in globals() and stream.active:
        #     stream.stop()
        #     stream.close()
        # Forcefully exit the program
        sys.exit(1)

    def on_event(self, result: RecognitionResult) -> None:
        sentence = result.get_sentence()
        if 'text' in sentence:
            recognized_text = sentence['text']
            print('RecognitionCallback text: ', recognized_text)
            
            # Check if "hi, Alicia" is in the recognized text (case insensitive)
            if "hi, alicia" in recognized_text.lower():
                self.wake_up = True
                # self.handle_alicia_wake_word()
                            # Extract part after wake word
                command_part = recognized_text.lower().split("hi, alicia", 1)[1].strip()
                # Process this as command
                self.process_command(command_part)
            
            # Also check for commands without wake word
            else:
                # Process the entire text as a potential command
                self.process_command(recognized_text.lower())
                
            if RecognitionResult.is_sentence_end(sentence):
                print(
                    'RecognitionCallback sentence end, request_id:%s, usage:%s'
                    % (result.get_request_id(), result.get_usage(sentence)))

    # Add a helper method to process commands
    def process_command(self, text):

        """Process text to extract grasp commands"""
        if any(word in text for word in ["grasp", "grab", "pick", "take", "抓取", "拿起", "green", "blue"]):
            print("=======================")
            # print(text)
            
            self.latest_command = text
            print(f"Extracted command: {self.latest_command}")
            print("=======================")
            # Create a publisher for commands if you want to broadcast them
            command_msg = String()
            command_msg.data = text
            try:
                # You would need to define this publisher in __init__
                self.command_pub.publish(command_msg)
                pass
            except Exception:
                pass


        # Function to handle when "hi, Alicia" is detected
    def handle_alicia_wake_word(self):
        """Handles actions when the wake phrase 'hi, Alicia' is detected"""
        print("Wake phrase 'hi, Alicia' detected!")
        
        # If ROS is initialized, publish to a wake_word topic
        try:
            if not rospy.is_shutdown():                # Create a Bool message
                wake_word_msg = Bool()
                # Set the data to True to indicate wake word activation
                wake_word_msg.data = True
                # Publish the message
                self.wake_word_pub.publish(wake_word_msg)
                # wake_word_pub.publish("activated")
                print("Published wake word activation message to ROS topic")
        except Exception as e:
            print(f"Failed to publish to ROS: {e}")
            
    def object_to_grasp(self, detected_objects):
        """
        Determine which objects to grasp based on speech command
        
        Args:
            detected_objects: List of detected objects with labels
            
        Returns:
            List of objects to grasp in specified order
        """
        # command = self.last_command.lower()
        # Use the latest command from speech recognition
        print("detected_objects", detected_objects)
        # while True:
        #     if self.latest_command:
        #         break

        if not self.latest_command:
            print("No command detected yet.")
            return detected_objects
        print("=======================")
        command = self.latest_command
        print(f"Processing command: '{command}'")

        print("=======================")
        # get the realtime text and
        ordered_objects = []
        
        # Empty command or no detected objects
        if not command or not detected_objects:
            return detected_objects  # Return all detected objects
        
        # Extract color words from command
        colors = []
        for word in command.split():
            clean_word = word.strip('.,!?;:')
            if clean_word in ["green", "blue", "red", "yellow", "Green", "Blue", "Red", "Yellow"]:
                colors.append(clean_word)
        print("color", colors)
        # If no colors specified, return all objects
        if not colors:
            return detected_objects
            
        # Match detected objects to specified colors in order
        for color in colors:
            for obj in detected_objects:
                if color.lower() in obj['label'].lower() and obj not in ordered_objects:
                    ordered_objects.append(obj)
                    print("ordered_objects", ordered_objects)
                    break
                    
        # If "only" appears in command and we found at least one match
        if "only" in command and ordered_objects:
            return ordered_objects
            
        # Return ordered objects if found, otherwise all detected objects
        return ordered_objects if ordered_objects else detected_objects
    

def signal_handler(sig, frame):
    print('Ctrl+C pressed, stop translation ...')
    # Stop translation
    recognition.stop()
    print('Translation stopped.')
    print(
        '[Metric] requestId: {}, first package delay ms: {}, last package delay ms: {}'
        .format(
            recognition.get_last_request_id(),
            recognition.get_first_package_delay(),
            recognition.get_last_package_delay(),
        ))
    # Forcefully exit the program
    sys.exit(0)


# # main function
# if __name__ == '__main__':
#     init_dashscope_api_key()
#     print('Initializing ...')

#     # Create the translation callback
#     callback = Callback()

#     # Call recognition service by async mode, you can customize the recognition parameters, like model, format,
#     # sample_rate For more information, please refer to https://help.aliyun.com/document_detail/2712536.html
#     recognition = Recognition(
#         model='paraformer-realtime-v2',
#         # 'paraformer-realtime-v1'、'paraformer-realtime-8k-v1'
#         format=format_pcm,
#         # 'pcm'、'wav'、'opus'、'speex'、'aac'、'amr', you can check the supported formats in the document
#         sample_rate=sample_rate,
#         # support 8000, 16000
#         semantic_punctuation_enabled=False,
#         callback=callback)

#     # Start translation
#     recognition.start()

#     signal.signal(signal.SIGINT, signal_handler)
#     print("Press 'Ctrl+C' to stop recording and translation...")
#     # Create a keyboard listener until "Ctrl+C" is pressed

#     while True:
#         if stream:
#             data = stream.read(3200, exception_on_overflow=False)
#             recognition.send_audio_frame(data)
#         else:
#             break

#     recognition.stop()