#!/usr/bin/env python3
"""
Demo script to test transformation logic locally without ROS/MQTT.
This simulates what happens inside the generated bridge code.
"""

class DotDict(dict):
    """A dictionary that allows dot notation access to its items."""
    def __getattr__(self, name):
        try:
            value = self[name]
            if isinstance(value, dict):
                return DotDict(value)
            return value
        except KeyError:
            raise AttributeError(name)


def test_temperature_transform():
    """Test temperature transformation (Celsius to Fahrenheit)"""
    print("\n=== Temperature Transform ===")
    
    # Simulate ROS message data
    raw_data = {"data": 25.0}  # 25°C
    msg = DotDict(raw_data)
    
    # Apply transformations
    transform = {
        "temperature_f": "msg.data * 1.8 + 32",
        "unit": "'fahrenheit'",
        "sensor_id": "'temp_sensor_01'"
    }
    
    result = {}
    for target, expr in transform.items():
        result[target] = eval(expr, {"msg": msg})
    
    print(f"Input: {raw_data}")
    print(f"Output: {result}")
    assert result["temperature_f"] == 77.0
    assert result["unit"] == "fahrenheit"
    print("✅ Temperature transform passed")


def test_humidity_transform():
    """Test humidity transformation with conditional logic"""
    print("\n=== Humidity Transform ===")
    
    raw_data = {"data": 0.65}  # 65% humidity
    msg = DotDict(raw_data)
    
    transform = {
        "humidity_percent": "msg.data * 100",
        "level": "'high' if msg.data > 0.7 else 'normal' if msg.data > 0.4 else 'low'"
    }
    
    result = {}
    for target, expr in transform.items():
        result[target] = eval(expr, {"msg": msg})
    
    print(f"Input: {raw_data}")
    print(f"Output: {result}")
    assert result["humidity_percent"] == 65.0
    assert result["level"] == "normal"
    print("✅ Humidity transform passed")


def test_battery_transform():
    """Test battery transformation with clamping"""
    print("\n=== Battery Transform ===")
    
    raw_data = {"data": 11.5}  # 11.5V
    msg = DotDict(raw_data)
    
    transform = {
        "percentage": "min(100, max(0, (msg.data / 12.0) * 100))",
        "voltage": "msg.data",
        "status": "'critical' if msg.data < 10.5 else 'low' if msg.data < 11.0 else 'normal'"
    }
    
    result = {}
    for target, expr in transform.items():
        result[target] = eval(expr, {"msg": msg})
    
    print(f"Input: {raw_data}")
    print(f"Output: {result}")
    assert 95 < result["percentage"] < 96
    assert result["status"] == "normal"
    print("✅ Battery transform passed")


def test_pose_transform():
    """Test pose transformation with nested fields"""
    print("\n=== Pose Transform ===")
    
    raw_data = {
        "x": 3.14159,
        "y": -2.71828,
        "theta": 1.5708  # 90 degrees in radians
    }
    msg = DotDict(raw_data)
    
    transform = {
        "x": "round(msg.x, 2)",
        "y": "round(msg.y, 2)",
        "heading": "round(msg.theta * 57.2958, 1)",
        "zone": "'north' if msg.y > 0 else 'south'"
    }
    
    result = {}
    for target, expr in transform.items():
        result[target] = eval(expr, {"msg": msg})
    
    print(f"Input: {raw_data}")
    print(f"Output: {result}")
    assert result["x"] == 3.14
    assert result["y"] == -2.72
    assert 89.9 < result["heading"] < 90.1
    assert result["zone"] == "south"
    print("✅ Pose transform passed")


if __name__ == "__main__":
    print("Testing Data Transformations")
    print("=" * 50)
    
    test_temperature_transform()
    test_humidity_transform()
    test_battery_transform()
    test_pose_transform()
    
    print("\n" + "=" * 50)
    print("✨ All transformation tests passed!")
