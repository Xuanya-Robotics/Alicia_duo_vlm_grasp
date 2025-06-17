# VLM Grasp for Alicia-D Manipulator

[中文版 (Chinese Version)](README_Ch.md)

This project enables Visual Language Model (VLM) based grasping capabilities for the Alicia-D robot. It utilizes the Bailian API for VLM processing.

## Prerequisites

1.  **Install Alicia_duo_ros1**:
    Refer to the installation guide at [https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git](https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git).

2.  **Register for Bailian API**:
    Sign up and obtain your API key from Alibaba Cloud Bailian: [https://cn.aliyun.com/product/bailian?from_alibabacloud=](https://cn.aliyun.com/product/bailian?from_alibabacloud=)
    *   **Important**: You will need to configure your Bailian API key. It's recommended to set it as an environment variable (e.g., `BAILIAN_API_KEY`) or store it in a configuration file that the application can access. Please refer to the Bailian SDK documentation for specific instructions on API key usage.

## Setup and Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/Xuanya-Robotics/Alicia_duo_vlm_grasp.git
    cd Alicia_duo_vlm_grasp
    ```

2.  **Create and activate a Python virtual environment**:
    ```bash
    python3 -m venv .avlm
    source .avlm/bin/activate
    ```
    *(To deactivate, simply type `deactivate`)*

3.  **Install required Python packages**:
    ```bash
    pip install -r requirements.txt
    ```

## Usage

1.  **Ensure your Bailian API key is configured** (e.g., as an environment variable).
    ```bash
    export BAILIAN_API_KEY="YOUR_API_KEY" 
    ```
    (Replace `"YOUR_API_KEY"` with your actual key. You might want to add this to your `.bashrc` or `.zshrc` for persistence.)

2.  **Run the main script**:
    ```bash
    python ./vlmgrasp/vlm_grasp.py
    ```

## Reference

*   Alibaba Cloud Bailian Speech Demo: [https://github.com/aliyun/alibabacloud-bailian-speech-demo](https://github.com/aliyun/alibabacloud-bailian-speech-demo)
*   Alicia Duo ROS1: [https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git](https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git)
