# Docker Support for Generated Bridge Code

RoboCon automatically includes a `Dockerfile` with your generated bridge code, pre-configured with all necessary dependencies for ROS or ROS2.

## What You Get

When you generate bridge code, the output directory contains:
- Generated Python bridge code
- `requirements.txt` with Python dependencies
- `safe_eval.py` utility
- **`Dockerfile`** - Ready-to-use Docker configuration (ROS or ROS2 based on your model)

## Quick Start

### 1. Generate Code (includes Dockerfile)

```bash
# Generate bridge code - Dockerfile is automatically included
robocon gen my_model.rbr -o gen
```

### 2. Build Docker Image

```bash
cd gen
docker build -t my-robot-bridge .

### 3. Run the Bridge

```bash
# Basic usage
docker run -v $(pwd):/workspace my-robot-bridge

# With host networking (recommended for broker connections)
docker run --network host -v $(pwd):/workspace my-robot-bridge
```

## Running Options
```

### With Network Configuration

If your bridge connects to an external broker (Redis, MQTT, etc.):

```bash
# Use host network mode
docker run --network host -v $(pwd)/gen:/workspace robocon-ros2-bridge

# Or expose specific ports and connect to broker
docker run -p 6379:6379 -v $(pwd)/gen:/workspace robocon-ros2-bridge
```

### Interactive Mode

For debugging or development:

```bash
docker run -it -v $(pwd)/gen:/workspace robocon-ros2-bridge /bin/bash
```

Then inside the container:
```bash
source /opt/ros/humble/setup.bash  # or /opt/ros/noetic/setup.bash for ROS
python3 MyRobot_bridges.py
```


## Complete Workflow Example

```bash
# 1. Generate bridge code (includes Dockerfile)
robocon gen my_model.rbr -o gen

# 2. Build Docker image
cd gen
docker build -t my-robot-bridge .

# 3. Run the bridge
docker run --network host -v $(pwd):/workspace my-robot-bridge
```

## Customization

### Custom Base Image

Modify the `FROM` directive in the Dockerfiles to use a different ROS distribution:

```dockerfile
FROM ros:humble-ros-base    # For ROS2 Humble
FROM ros:noetic-ros-base # For ROS Noetic
```

### Additional Dependencies

Add ROS packages or Python libraries as needed:

```dockerfile
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    custom-package
```

### Environment Variables

Set environment variables for your bridge configuration:

```bash
docker run -e BROKER_HOST=192.168.1.100 \
           -e BROKER_PORT=6379 \
           -v $(pwd)/gen:/workspace \
           robocon-ros2-bridge
```

## Troubleshooting

### Bridge Not Starting

- Ensure the generated code is in `/workspace`
- Check that all required ROS messages are available
- Verify broker connectivity

### Permission Issues

If you encounter permission issues with mounted volumes:

```bash
docker run --user $(id -u):$(id -g) -v $(pwd)/gen:/workspace robocon-ros2-bridge
```

### Debugging

View container logs:
```bash
docker logs <container_id>
```

Access running container:
```bash
docker exec -it <container_id> /bin/bash
```
