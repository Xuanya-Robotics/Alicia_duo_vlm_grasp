#!/usr/bin/env python3
# Copyright (C) Alibaba Group. All Rights Reserved.
# MIT License (https://opensource.org/licenses/MIT)

import os
import sys
import threading

import dashscope
from dashscope.audio.tts_v2 import *

sys.path.append(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 './python'))

from RealtimeMp3Player import RealtimeMp3Player


class TextToSpeech:
    """Class for text-to-speech synthesis and playback using DashScope API"""
    
    def __init__(self, api_key=None):
        """Initialize the TTS engine with API key"""
        # Set the API key
        if api_key:
            dashscope.api_key = api_key
        elif 'DASHSCOPE_API_KEY' in os.environ:
            dashscope.api_key = os.environ['DASHSCOPE_API_KEY']
        else:
            dashscope.api_key = 'sk-82d96b55ac664519b5d64c7da3ca8e60'  # Default placeholder
    
    def synthesize_and_play(self, text, model='cosyvoice-v1', voice='loongstella', save_path=None):
        """
        Synthesize speech from text and play it in real-time
        
        Args:
            text: The text to synthesize
            model: TTS model to use
            voice: Voice to use for synthesis
            save_path: Path to save the MP3 file (None = only play without saving)
        """
        player = RealtimeMp3Player()
        player.start()
        
        complete_event = threading.Event()
        file_path = save_path if save_path else 'result.mp3'
        
        class Callback(ResultCallback):
            def __init__(self, parent):
                self.parent = parent
                self.file = None
                
            def on_open(self):
                if save_path is not None:
                    self.file = open(file_path, 'wb')
                print('TTS websocket connection opened')

            def on_complete(self):
                print('Speech synthesis completed successfully')
                complete_event.set()

            def on_error(self, message: str):
                print(f'Speech synthesis failed: {message}')
                complete_event.set()

            def on_close(self):
                if self.file:
                    self.file.close()
                print('TTS websocket connection closed')

            def on_event(self, message):
                pass

            def on_data(self, data: bytes) -> None:
                player.write(data)
                if self.file:
                    self.file.write(data)
        
        # Initialize callback and synthesizer
        synthesizer_callback = Callback(self)
        speech_synthesizer = SpeechSynthesizer(
            model=model,
            voice=voice,
            callback=synthesizer_callback
        )
        
        # Synthesize and play
        print(f'Synthesizing: "{text[:50]}{"..." if len(text) > 50 else ""}"')
        speech_synthesizer.call(text)
        
        # Wait for completion
        complete_event.wait()
        
        # Clean up
        player.stop()
        
        # Return metrics
        return {
            'request_id': speech_synthesizer.get_last_request_id(),
            'first_package_delay_ms': speech_synthesizer.get_first_package_delay()
        }


# Example usage
if __name__ == '__main__':
    # Sample text
    sample_text = '想不到时间过得这么快！昨天和你视频聊天，看到你那自豪又满意的笑容，我的心里呀，就如同喝了一瓶蜜一样甜呢！真心为你开心呢！'
    
    # Create TTS instance
    tts = TextToSpeech()
    
    # Synthesize and play the text
    metrics = tts.synthesize_and_play(sample_text)
    
    # Print metrics
    print(f'Request ID: {metrics["request_id"]}')
    print(f'First package delay: {metrics["first_package_delay_ms"]} ms')