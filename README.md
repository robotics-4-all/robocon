# RoboConnect

**RoboConnect** is a powerful Domain-Specific Language (DSL) designed to simplify the integration of ROS (Robot Operating System) and ROS 2 systems with IoT and Smart Environment infrastructures. It provides a high-level, declarative syntax to define communication bridges between robotics middleware and message brokers like MQTT, Redis, and AMQP.

---

## 🚀 Key Features

- **Unified Syntax**: A consistent, human-readable language for both ROS 1 and ROS 2.
- **Multi-Protocol Support**: Seamlessly bridge to **MQTT**, **Redis**, and **AMQP** (RabbitMQ).
- **Smart Bridging**:
    - **Implicit Direction**: Automatically determines `R2B` (ROS to Broker) or `B2R` (Broker to ROS) based on endpoint definition.
    - **NodeBridge**: Automatically expand all publishers, subscribers, services, and actions of a ROS Node into bridges with a single line.
- **Comprehensive Endpoint Support**:
    - **Topics**: Pub/Sub telemetry and commands.
    - **Services**: Request/Response remote procedure calls.
    - **Actions**: Goal-oriented task management (ROS 2).
    - **TF**: Coordinate frame transformations.
- **Validation & Generation**: Built-in CLI for model validation and automated Python code generation.

---

## 📦 Installation

### From Source
```bash
# Clone the repository
git clone https://github.com/robotics-4-all/RoboConnect.git
cd RoboConnect

# Install in editable mode
pip install -e .
```

### Using Docker
```bash
# Build the image
make docker

# Run the API/CLI container
docker run -it roboconnect-api roboconnect --help
```

---

## 🛠 DSL Reference

### 1. Robot Definition
Define your ROS system, its topics, services, and nodes.

```textx
Robot[ROS2] MyRobot @ "192.168.1.10" {
    TOPICS
        odom [ "nav_msgs/Odometry", "/odom" ]
        cmd_vel [ "geometry_msgs/Twist", "/cmd_vel" ]
    
    SERVICES
        reset [ "std_srvs/Empty", "/reset" ]

    NODES
        base_controller {
            publishes: [ odom ]
            subscribes: [ cmd_vel ]
            services: [ reset ]
        }
}
```

### 2. Broker Configuration
Configure the target message broker.

```textx
BROKER[MQTT] LocalBroker {
    host: "localhost",
    port: 1883,
    auth.username: "admin",
    auth.password: "secret"
}
```

### 3. Bridge Definitions

#### Individual Bridges
```textx
# Topic Bridge (ROS -> Broker)
Bridge[Topic] odom_bridge odom:"robot/odom";

# Service Bridge (Broker -> ROS)
Bridge[Service] reset_bridge "robot/reset":reset;
```

#### The NodeBridge (Recommended)
Automatically bridge all endpoints defined in a `ROSNode`.
- `publishes` -> `R2B` Topic Bridge
- `subscribes` -> `B2R` Topic Bridge
- `services` -> `B2R` Service Bridge
- `actions` -> `B2R` Action Bridge

```textx
# Bridges everything in base_controller with a "robot" prefix
Bridge[Node] controller_bridges base_controller:"robot";
```

---

## 💻 CLI Usage

RoboConnect comes with a command-line interface for managing your models.

### Validate a Model
Verify the syntax and references in your `.rbr` file.
```bash
roboconnect validate examples/complex_model.rbr
```

### Generate Code
Generate the Python bridge implementation. The generator is automatically detected from the model's `Robot[ROS]` or `Robot[ROS2]` definition.

```bash
# Auto-detect from model (recommended)
roboconnect gen examples/complex_model.rbr -o ./gen

# Explicitly specify generator (optional)
roboconnect gen examples/complex_model.rbr ros2 -o ./gen
roboconnect gen examples/ros_model.rbr ros -o ./gen
```

---

## 📂 Project Structure

- `roboconnect/grammar/`: textX grammar definitions (`.tx`).
- `roboconnect/templates/`: Jinja2 templates for code generation (`.j2`).
- `roboconnect/m2t/`: Model-to-Text transformation logic.
- `examples/`: Collection of ROS 1 and ROS 2 system models.
- `scripts/`: Utility scripts for bulk validation and testing.

---

## 🧪 Testing & Quality Assurance

We provide automated scripts to ensure model validity and generator correctness.

- **Validate All Examples**:
  ```bash
  python scripts/validate_examples.py
  ```
- **Test Code Generation**:
  ```bash
  python scripts/test_generation.py
  ```

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.