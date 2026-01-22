# Robocon

**Robocon** is a powerful Domain-Specific Language (DSL) designed to simplify the integration of ROS (Robot Operating System) and ROS 2 systems with IoT and CPS environments and systems. It provides a high-level, declarative syntax to define communication bridges between robotics middleware and message brokers like MQTT, Redis, and AMQP.

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
    <!-- - **Actions**: Goal-oriented task management (ROS 2). -->
    - **TF**: Coordinate frame transformations.
- **Validation & Generation**: Built-in CLI for model validation and automated Python code generation.

### 🆕 Advanced Features (New!)

#### 🔄 Data Transformation & Semantic Mapping
Transform message data in real-time with Python expressions using **safe, sandboxed evaluation**:

```textx
Bridge[Topic] temp_bridge temp_sensor : "iot/temperature" {
    transform: {
        temp_f: "msg.data * 1.8 + 32",
        unit: "'fahrenheit'",
        severity: "'HIGH' if msg.data > 100 else 'NORMAL'"
    }
};
```

**Features:**
- **Dot notation**: Access nested fields with `msg.data`, `msg.field.subfield`
- **Python expressions**: Math, conditionals, string operations
- **Safe functions**: `round()`, `min()`, `max()`, `len()`, string methods
- **Secure**: AST-based sandboxing blocks dangerous operations

#### 🎯 Conditional Bridging (Filtering)
Filter messages based on runtime conditions to reduce bandwidth and processing:

```textx
// Only forward critical errors (whitelist)
Bridge[Topic] alerts diagnostics : "alerts/critical" {
    when: "msg.level == 'ERROR' or msg.level == 'FATAL'"
};

// Filter out test data (blacklist)
Bridge[Topic] prod_data sensors : "iot/data" {
    unless: "msg.source == 'test'"
};

// Combine filtering with transformation
Bridge[Topic] battery_alerts battery : "alerts/battery" {
    when: "msg.voltage < 11.0"
    transform: {
        percentage: "round((msg.voltage / 12.0) * 100, 1)",
        severity: "'CRITICAL' if msg.voltage < 10.0 else 'WARNING'"
    }
};
```

**Features:**
- **`when` clause**: Forward only if condition is true (whitelist)
- **`unless` clause**: Forward unless condition is true (blacklist)
- **Mutually exclusive**: Use either `when` or `unless`, not both
- **Combine with transforms**: Apply transformations only to filtered messages

#### 🔒 Secure Expression Evaluation
All transformations and conditions use **sandboxed, AST-validated evaluation**:
- ✅ **Whitelist approach**: Only explicitly safe operations allowed
- ❌ **Blocks**: File I/O, imports, code execution, system calls
- 🛡️ **Safe by default**: No code injection vulnerabilities
- �� **Full test coverage**: 29 comprehensive security tests

See `examples/transformations/` and `examples/conditional_bridging/` for complete examples.


---

## 📦 Installation

### From Source
```bash
# Clone the repository
git clone https://github.com/robotics-4-all/Robocon.git
cd Robocon

# Install in editable mode
pip install -e .
```

### Using Docker
```bash
# Build the image
make docker-build

# Run the API/CLI container
make docker-run
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
Broker[MQTT] LocalBroker {
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
- `publishes` → `R2B` Topic Bridge
- `subscribes` → `B2R` Topic Bridge
- `services` → `B2R` Service Bridge
- `actions` → `B2R` Action Bridge

**Basic Usage (with prefix):**
```textx
# Bridges everything in base_controller with a "robot" prefix
# Topics: robot/odom, robot/cmd_vel
# Services: robot/reset
Bridge[Node] controller_bridges base_controller:"robot";
```

**Without Prefix:**
```textx
# Uses topic/service/action names directly (no prefix)
Bridge[Node] my_bridge base_controller;
```

**Custom URI Mappings:**
Override the default prefix-based URIs with custom broker URIs for specific topics, services, or actions:

```textx
Bridge[Node] advanced_bridge base_controller:"robot" {
    topic_maps: {
        odom: "telemetry/robot/odometry",
        cmd_vel: "commands/robot/velocity"
    },
    service_maps: {
        reset: "services/robot/emergency_reset"
    }
};
# Result:
# - odom → "telemetry/robot/odometry" (custom)
# - cmd_vel → "commands/robot/velocity" (custom)
# - reset → "services/robot/emergency_reset" (custom)
# - Other unmapped items use prefix: "robot/{name}"
```

See `examples/nodebridge_features.rbr` for comprehensive examples.

---

## 💻 CLI Usage

Robocon comes with a command-line interface for managing your models.

### Validate a Model
Verify the syntax and references in your `.rbr` file.
```bash
robocon validate examples/complex_model.rbr
```

### Generate Code
Generate the Python bridge implementation. The generator is automatically detected from the model's `Robot[ROS]` or `Robot[ROS2]` definition.

```bash
# Auto-detect from model (recommended)
robocon gen examples/complex_model.rbr -o ./gen

# Explicitly specify generator (optional)
robocon gen examples/complex_model.rbr ros2 -o ./gen
robocon gen examples/ros_model.rbr ros -o ./gen
```

---

## 📂 Project Structure

- `robocon/grammar/`: textX grammar definitions (`.tx`).
- `robocon/templates/`: Jinja2 templates for code generation (`.j2`).
- `robocon/m2t/`: Model-to-Text transformation logic.
- `examples/`: Collection of ROS 1 and ROS 2 system models.
- `scripts/`: Utility scripts for bulk validation and testing.
- `api/`: API server for model validation and code generation.

---

## 🧪 Testing & Quality Assurance

Robocon has a comprehensive test suite with **96%+ code coverage** using pytest.

### Running Tests

```bash
# Install test dependencies
pip install -e ".[test]"

# Run all tests
pytest tests/

# Run tests with coverage report
pytest tests/ --cov=robocon --cov-report=term-missing

# Run tests with HTML coverage report
pytest tests/ --cov=robocon --cov-report=html
# Open htmlcov/index.html in your browser

# Run tests in Docker container (requires Docker)
make test

# Run specific test modules
pytest tests/test_utils.py -v
pytest tests/test_m2t/ -v
pytest tests/integration/ -v
```

### Test Structure

```
tests/
├── conftest.py              # Shared fixtures and test configuration
├── test_language.py         # Language and metamodel tests
├── test_utils.py            # Utility functions and processors
├── test_generator.py        # Generator entry points
├── test_cli.py              # CLI commands
├── test_api.py              # FastAPI endpoints
├── test_m2t/                # Model-to-text transformation tests
│   ├── test_rosgen.py      # ROS code generator tests
│   └── test_ros2gen.py     # ROS2 code generator tests
└── integration/             # End-to-end integration tests
    ├── test_examples.py    # Tests using example models
    └── test_end_to_end.py  # Complete workflow tests
```

### Example Validation & Generation

We also provide automated scripts to ensure model validity and generator correctness.

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