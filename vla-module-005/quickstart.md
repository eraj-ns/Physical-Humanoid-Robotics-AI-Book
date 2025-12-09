---
id: specs-005-vla-module-quickstart
---
# Quickstart: Module 4 Labs

This guide provides instructions on how to set up the environment for the labs and code examples in Module 4.

## 1. Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble**: Installed and configured.
- **Python 3.10+**
- **OpenAI API Key**: For using Whisper and LLM services.
- **Microphone**: For voice command input.

## 2. Environment Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Install Python dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3.  **Build the ROS 2 packages**:
    ```bash
    cd labs/module4
    colcon build
    source install/setup.bash
    ```

4.  **Set your OpenAI API Key**:
    ```bash
    export OPENAI_API_KEY='your_api_key_here'
    ```

## 3. Running the Labs

Each lab has its own launch file and instructions in the corresponding chapter.

### Example: Running the Voice-to-Action Lab

```bash
# In one terminal, launch the robot simulation
ros2 launch <simulation_package> <simulation_launch_file>.py

# In another terminal, launch the VLA pipeline
ros2 launch vla_pipeline vla_pipeline.launch.py
```

## 4. Troubleshooting

- **Microphone Issues**: Use a tool like `arecord` to test your microphone and ensure it is working correctly.
- **ROS 2 Issues**: Make sure that your ROS 2 environment is sourced correctly.
- **OpenAI API Issues**: Check your API key and ensure you have a working internet connection.
