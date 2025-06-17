# Alicia-D 机械臂 VLM 抓取

本项目为 Alicia-D 机器人提供基于视觉语言模型 (VLM) 的抓取能力。它利用百炼 API 进行 VLM 处理。

## 先决条件

1.  **安装 Alicia_duo_ros1**:
    安装指南：[https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git](https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git)

2.  **注册百炼 API**:
    请从阿里云百炼注册并获取API 密钥：[https://cn.aliyun.com/product/bailian?from_alibabacloud=](https://cn.aliyun.com/product/bailian?from_alibabacloud=)
    *   **重要提示**：请配置百炼 API 密钥。建议将其设置为环境变量（例如 `BAILIAN_API_KEY`）或将其存储在应用程序可以访问的配置文件中。有关 API 密钥使用的具体说明，请参阅百炼 SDK 文档。

## 设置与安装

1.  **克隆代码仓库**:
    ```bash
    git clone https://github.com/Xuanya-Robotics/Alicia_duo_vlm_grasp.git
    cd Alicia_duo_vlm_grasp
    ```

2.  **创建并激活 Python 虚拟环境**:
    ```bash
    python3 -m venv .avlm
    source .avlm/bin/activate
    ```
    *（若要停用虚拟环境，只需键入 `deactivate`）*

3.  **安装所需的 Python 包**:
    ```bash
    pip install -r requirements.txt
    ```

## 使用方法

1.  **确保百炼 API 密钥已配置**（例如，作为环境变量）。
    ```bash
    export BAILIAN_API_KEY="实际的API密钥"
    ```
    （请将 `"实际的API密钥"` 替换为真实的密钥。可将其添加到 `.bashrc` 或 `.zshrc` 文件中以便持久化。）

2.  **运行主脚本**:
    ```bash
    python ./vlmgrasp/vlm_grasp.py
    ```

## 参考资料

*   阿里云百炼语音: [https://github.com/aliyun/alibabacloud-bailian-speech-demo](https://github.com/aliyun/alibabacloud-bailian-speech-demo)
*   Alicia Duo ROS1: [https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git](https://github.com/Xuanya-Robotics/Alicia_duo_ros1.git)