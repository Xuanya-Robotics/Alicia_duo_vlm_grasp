import os
import base64
import cv2
import numpy as np
import ast
import json
import uuid
import tempfile




class ObjectDetection:
    """Class for detecting objects in images using vision-language models"""
    
    def __init__(self, api_key=None):
        """Initialize the object detector"""
        # Clear proxy settings
        os.environ['http_proxy'] = ''
        os.environ['https_proxy'] = ''
        os.environ['HTTP_PROXY'] = ''
        os.environ['HTTPS_PROXY'] = ''
        os.environ['no_proxy'] = '*'
        os.environ['NO_PROXY'] = '*'
        
        # Set API key
        if api_key:
            os.environ['DASHSCOPE_API_KEY'] = api_key
        elif 'DASHSCOPE_API_KEY' not in os.environ:
            os.environ['DASHSCOPE_API_KEY'] = 'your api key'
    
    def encode_image(self, image_path):
        """Encode image to base64 format"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    def inference_with_api(self, image_path, prompt, sys_prompt="You are a helpful assistant.", 
                           model_id="qwen2.5-vl-72b-instruct", min_pixels=512*28*28, max_pixels=2048*28*28):
        """Call API for inference"""
        import requests
        
        # Encode image
        base64_image = self.encode_image(image_path)
        
        # Build API request
        headers = {
            "Authorization": f"Bearer {os.getenv('DASHSCOPE_API_KEY')}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": model_id,
            "messages": [
                {
                    "role": "system",
                    "content": [{"type":"text", "text": sys_prompt}]
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "min_pixels": min_pixels,
                            "max_pixels": max_pixels,
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
                        },
                        {"type": "text", "text": prompt}
                    ]
                }
            ]
        }
        
        try:
            # Send request directly using requests
            response = requests.post(
                "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions",
                headers=headers,
                json=payload,
                proxies={"http": "", "https": ""}
            )
            
            if response.status_code == 200:
                result = response.json()
                return result["choices"][0]["message"]["content"]
            else:
                print(f"API Error: HTTP {response.status_code}")
                print(f"Error content: {response.text}")
                
                # Try alternative API format
                alt_headers = {
                    "Authorization": f"Bearer {os.getenv('DASHSCOPE_API_KEY')}",
                    "Content-Type": "application/json"
                }
                
                alt_payload = {
                    "model": "qwen-vl-plus",
                    "input": {
                        "messages": [
                            {
                                "role": "system",
                                "content": sys_prompt
                            },
                            {
                                "role": "user", 
                                "content": [
                                    {
                                        "image": base64_image
                                    },
                                    {
                                        "text": prompt
                                    }
                                ]
                            }
                        ]
                    }
                }
                
                response = requests.post(
                    "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation",
                    headers=alt_headers,
                    json=alt_payload,
                    proxies={"http": "", "https": ""}
                )
                
                if response.status_code == 200:
                    result = response.json()
                    return result["output"]["choices"][0]["message"]["content"]
                else:
                    return f"API Error: HTTP {response.status_code}\n{response.text}"
        
        except Exception as e:
            print(f"Request exception: {e}")
            return f"Request failed: {str(e)}"

    def parse_json(self, json_output):
        """Parse JSON from response string"""
        try:
            # Try to extract JSON block
            start_idx = json_output.find("[")
            end_idx = json_output.rfind("]") + 1
            
            if start_idx >= 0 and end_idx > start_idx:
                json_str = json_output[start_idx:end_idx]
                return json_str
            else:
                # Look for code block
                if "```json" in json_output:
                    lines = json_output.splitlines()
                    start = False
                    json_lines = []
                    
                    for line in lines:
                        if line.strip() == "```json":
                            start = True
                            continue
                        elif line.strip() == "```" and start:
                            break
                        elif start:
                            json_lines.append(line)
                    
                    if json_lines:
                        return "\n".join(json_lines)
                
                return json_output
        except Exception as e:
            print(f"JSON parsing error: {e}")
            return json_output

    def smart_resize(self, height, width, min_pixels, max_pixels):
        """Resize image dimensions to meet pixel limits"""
        original_pixels = height * width
        
        if original_pixels <= max_pixels and original_pixels >= min_pixels:
            return height, width
        
        if original_pixels < min_pixels:
            scale = (min_pixels / original_pixels) ** 0.5
            return int(height * scale), int(width * scale)
        
        if original_pixels > max_pixels:
            scale = (max_pixels / original_pixels) ** 0.5
            return int(height * scale), int(width * scale)

    def extract_center_coordinates(self, response, image_height, image_width, input_height, input_width):
        """Extract center coordinates from API response"""
        # Parse JSON response
        json_str = self.parse_json(response)
        
        # Try to parse JSON
        try:
            try:
                json_output = json.loads(json_str)
            except json.JSONDecodeError:
                json_output = ast.literal_eval(json_str)
            
            if not isinstance(json_output, list):
                if isinstance(json_output, dict):
                    json_output = [json_output]
                else:
                    raise ValueError("Cannot convert response to list")
        except Exception as e:
            print(f"JSON parsing error: {e}")
            # Create default bounding box
            json_output = [{
                "label": "Green Cube",
                "bbox_2d": [image_width/3, image_height/3, image_width*2/3, image_height*2/3]
            }]
        
        # List to store center coordinates
        center_coordinates = []
        
        # Calculate center for each bounding box
        for bbox in json_output:
            if "bbox_2d" not in bbox:
                print(f"Warning: bbox missing bbox_2d field: {bbox}")
                continue
                
            try:
                coords = bbox["bbox_2d"]
                
                # Check if coordinates are normalized (all values < 10)
                if all(c < 10 for c in coords):
                    abs_x1 = int(float(coords[0]) * image_width)
                    abs_y1 = int(float(coords[1]) * image_height)
                    abs_x2 = int(float(coords[2]) * image_width)
                    abs_y2 = int(float(coords[3]) * image_height)
                else:
                    # Assume pixel coordinates but convert from input size to display size
                    abs_x1 = int(float(coords[0]) / input_width * image_width)
                    abs_y1 = int(float(coords[1]) / input_height * image_height)
                    abs_x2 = int(float(coords[2]) / input_width * image_width)
                    abs_y2 = int(float(coords[3]) / input_height * image_height)
                    
                # Ensure x1 <= x2, y1 <= y2
                if abs_x1 > abs_x2:
                    abs_x1, abs_x2 = abs_x2, abs_x1
                if abs_y1 > abs_y2:
                    abs_y1, abs_y2 = abs_y2, abs_y1
                
                # Calculate center coordinates
                center_x = (abs_x1 + abs_x2) // 2
                center_y = (abs_y1 + abs_y2) // 2
                
                # Store center coordinates with label
                center_coordinates.append({
                    "label": bbox.get("label", "Object"),
                    "center": [center_x, center_y],
                    "bbox": [abs_x1, abs_y1, abs_x2, abs_y2]  # Store full bbox for visualization
                })
                    
            except Exception as e:
                print(f"Coordinate calculation error: {e}")
                continue
        
        return center_coordinates

    def detect_green_cube(self, image, prompt=None):
        """
        Detect green cube in an OpenCV image and return center coordinates
        
        Args:
            image: OpenCV image (numpy array from cv2.imread)
            
        Returns:
            List of center coordinates for detected green cubes
        """
        # Check if image is valid
        if image is None or image.size == 0:
            return {"error": "Invalid image"}
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Create temp file to save the image
        temp_dir = tempfile.gettempdir()
        temp_filename = f"{uuid.uuid4()}.jpg"
        temp_path = os.path.join(temp_dir, temp_filename)
        
        try:
            # Save image to temp file
            cv2.imwrite(temp_path, image)
            
            # Calculate input dimensions for API
            min_pixels = 512*28*28
            max_pixels = 2048*28*28
            input_height, input_width = self.smart_resize(height, width, min_pixels, max_pixels)
            
            if prompt is None:
                # Define prompt
                prompt = "Outline the blue and green cubes in the image and output coordinates in JSON format: [{\"label\": \"Green Cube\", \"bbox_2d\": [x1, y1, x2, y2]}]"
                # prompt = "Outline the green cube in the image and output coordinates in JSON format: [{\"label\": \"Green Cube\", \"bbox_2d\": [x1, y1, x2, y2]}]"
            
            # Call API for inference
            print("Calling API for cube detection...")
            response = self.inference_with_api(temp_path, prompt, min_pixels=min_pixels, max_pixels=max_pixels)
            print(f"API Response: {response}")
            
            # Extract center coordinates
            center_coordinates = self.extract_center_coordinates(response, height, width, input_height, input_width)
            
            return center_coordinates
            
        except Exception as e:
            print(f"Error in detection: {e}")
            return []
        finally:
            # Clean up temp file
            if os.path.exists(temp_path):
                try:
                    os.remove(temp_path)
                except:
                    pass
    
    def detect_objects(self, image, object_type=None):
        """
        Detect objects in an OpenCV image and return center coordinates
        
        Args:
            image: OpenCV image (numpy array from cv2.imread)
            object_type: Optional type of object to detect (e.g., "green cube", "red ball")
            
        Returns:
            List of center coordinates for detected objects
        """
        # Check if image is valid
        if image is None or image.size == 0:
            return {"error": "Invalid image"}
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Create temp file to save the image
        temp_dir = tempfile.gettempdir()
        temp_filename = f"{uuid.uuid4()}.jpg"
        temp_path = os.path.join(temp_dir, temp_filename)
        
        try:
            # Save image to temp file
            cv2.imwrite(temp_path, image)
            
            # Calculate input dimensions for API
            min_pixels = 512*28*28
            max_pixels = 2048*28*28
            input_height, input_width = self.smart_resize(height, width, min_pixels, max_pixels)
            
            # Define prompt based on object_type
            if object_type:
                prompt = f"Outline the {object_type} in the image and output coordinates in JSON format: [{{\"label\": \"{object_type}\", \"bbox_2d\": [x1, y1, x2, y2]}}]"
            else:
                prompt = "Outline all objects in the image and output coordinates in JSON format: [{\"label\": \"Object Name\", \"bbox_2d\": [x1, y1, x2, y2]}]"
            
            # Call API for inference
            print(f"Calling API for object detection...")
            response = self.inference_with_api(temp_path, prompt, min_pixels=min_pixels, max_pixels=max_pixels)
            
            # Extract center coordinates
            center_coordinates = self.extract_center_coordinates(response, height, width, input_height, input_width)
            
            return center_coordinates
            
        except Exception as e:
            print(f"Error in detection: {e}")
            return []
        finally:
            # Clean up temp file
            if os.path.exists(temp_path):
                try:
                    os.remove(temp_path)
                except:
                    pass
    
    def visualize_results(self, image, detections):
        """
        Visualize detection results on image
        
        Args:
            image: OpenCV image
            detections: List of detection results with center coordinates
            
        Returns:
            Image with visualization
        """
        result_image = image.copy()
        
        for obj in detections:
            # Draw bounding box if available
            if "bbox" in obj:
                x1, y1, x2, y2 = obj["bbox"]
                cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw center point
            center = obj["center"]
            cv2.circle(result_image, (center[0], center[1]), 5, (0, 0, 255), -1)
            
            # Draw label
            label = obj.get("label", "Object")
            cv2.putText(result_image, label, 
                      (center[0] - 50, center[1] - 10),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Save visualization result
        output_path = os.path.join('./', f"detection_result_{uuid.uuid4()}.jpg")
        cv2.imshow("Detection Result", result_image)
        cv2.waitKey(10000)
        cv2.imwrite(output_path, result_image)
        print(f"Visualization saved to: {output_path}")
        
        # Try to display image if in interactive environment
        try:
            cv2.imshow("Detection Result", result_image)
            cv2.waitKey(1)  # Show without blocking in ROS environment
        except:
            pass
            
        return result_image


# Example usage
if __name__ == "__main__":
    # Create object detector
    detector = ObjectDetection()
    
    # Load image with OpenCV
    example_image_path = "./7.jpg"
    image = cv2.imread(example_image_path)
    
    if image is not None:
        # Detect green cube
        center_coordinates = detector.detect_green_cube(image)
        print("Center Coordinates:", center_coordinates)
        
        # Visualize results
        result_image = detector.visualize_results(image, center_coordinates)
        
        # Show result (will be displayed by visualize_results)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"Failed to load image at {example_image_path}")