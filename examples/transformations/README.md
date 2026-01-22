# Data Transformation Example

This example demonstrates the **Data Transformation & Semantic Mapping** feature in RoboCon DSL.

## Overview

The `temp_monitor.rbr` model shows a temperature monitoring robot that bridges ROS topics to an MQTT cloud platform with real-time data transformations.

## Key Features Demonstrated

### 1. Unit Conversion
```robocon
Bridge[Topic] temp_bridge temp_celsius : "sensors/temp" {
    transform: {
        temperature_f: "msg.data * 1.8 + 32",
        unit: "'fahrenheit'"
    }
};
```
Converts Celsius to Fahrenheit for US-based systems.

### 2. Data Enrichment
```robocon
Bridge[Topic] humidity_bridge humidity : "sensors/humidity" {
    transform: {
        humidity_percent: "msg.data * 100",
        level: "'high' if msg.data > 0.7 else 'normal' if msg.data > 0.4 else 'low'"
    }
};
```
Adds percentage values and categorical classifications.

### 3. Range Clamping
```robocon
transform: {
    percentage: "min(100, max(0, (msg.data / 12.0) * 100))"
}
```
Ensures values stay within valid ranges.

### 4. Precision Control
```robocon
transform: {
    x: "round(msg.x, 2)",
    heading: "round(msg.theta * 57.2958, 1)"
}
```
Reduces payload size by limiting decimal places.

### 5. Conditional Logic
```robocon
transform: {
    status: "'critical' if msg.data < 10.5 else 'low' if msg.data < 11.0 else 'normal'"
}
```
Applies business logic to categorize data.

## Running the Example

### 1. Generate the Bridge Code
```bash
cd examples/transformations
robocon-cli generate temp_monitor.rbr --target ros --output generated/
```

### 2. Run the Bridge
```bash
python TempMonitorBot_bridge.py
```

### 3. Expected Output

The bridge will:
- Subscribe to MQTT topic `sensors/temp` and receive transformed data like:
  ```json
  {
    "temperature_f": 77.0,
    "unit": "fahrenheit",
    "sensor_id": "temp_sensor_01"
  }
  ```
- Apply transformations in real-time as ROS messages arrive
- Publish enriched data to the cloud platform

## Transformation Syntax

All transformations support:
- **Dot notation**: `msg.data`, `msg.nested.field`
- **Python expressions**: Math operations, conditionals, functions
- **Built-in functions**: `min()`, `max()`, `round()`, `hasattr()`
- **String literals**: Use single quotes `'value'`
- **Nested access**: `msg.header.stamp.secs`

## Notes

- Transformations are evaluated at runtime using Python's `eval()`
- The `DotDict` helper enables dot notation access to message fields
- Invalid expressions will be logged as errors without crashing the bridge
- For security, only use transformations with trusted models
