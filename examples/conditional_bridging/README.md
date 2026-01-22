# Conditional Bridging (Filtering) Example

This example demonstrates the **Conditional Bridging** feature using `when` and `unless` clauses to filter messages based on runtime conditions.

## Overview

The `smart_sensor.rbr` model shows a smart IoT sensor that only forwards relevant data to the cloud, reducing bandwidth usage and cloud costs by filtering out normal operational data.

## Key Features Demonstrated

### 1. `when` Clause - Include If True
```robocon
Bridge[Topic] critical_diagnostics diagnostics : "alerts/critical" {
    when: "msg.data == 'ERROR' or msg.data == 'FATAL'"
};
```
Only forwards messages when the condition evaluates to `True`.

### 2. `unless` Clause - Exclude If True
```robocon
Bridge[Topic] production_status status : "system/status" {
    unless: "msg.data.startswith('DEBUG') or msg.data.startswith('TEST')"
};
```
Forwards all messages **except** when the condition is `True`.

### 3. Numeric Thresholds
```robocon
Bridge[Topic] high_temp_alerts temperature : "alerts/temperature" {
    when: "msg.data > 85.0"
};
```
Filter based on numeric values.

### 4. Range Checks
```robocon
Bridge[Topic] vibration_alerts vibration : "alerts/vibration" {
    when: "msg.data > 2.5 or msg.data < 0.1"
};
```
Detect values outside normal operating ranges.

### 5. Combining Conditions with Transformations
```robocon
Bridge[Topic] low_battery_alerts battery : "alerts/battery" {
    when: "msg.data < 11.0"
    transform: {
        voltage: "msg.data",
        percentage: "round((msg.data / 12.0) * 100, 1)",
        severity: "'CRITICAL' if msg.data < 10.0 else 'WARNING'"
    }
};
```
Apply transformations only to messages that pass the filter.

## Benefits

### Reduced Bandwidth
- Only send critical data to the cloud
- Filter out normal operational messages
- Save on data transfer costs

### Lower Cloud Costs
- Fewer messages = lower ingestion costs
- Reduced storage requirements
- Optimized processing pipelines

### Improved Signal-to-Noise Ratio
- Focus on actionable alerts
- Eliminate debug/test data from production
- Simplify downstream analytics

## Running the Example

### 1. Generate the Bridge Code
```bash
cd examples/conditional_bridging
robocon-cli generate smart_sensor.rbr --target ros --output generated/
```

## Expected Behavior

### Messages That Pass
- Diagnostics: `"ERROR"`, `"FATAL"`
- Temperature: `90.5`, `100.0`, `85.1`
- Vibration: `3.0`, `0.05`
- Battery: `10.8V`, `9.5V`
- Status: `"RUNNING"`, `"IDLE"`

### Messages That Are Filtered
- Diagnostics: `"INFO"`, `"WARN"`
- Temperature: `70.0`, `80.0`, `85.0`
- Vibration: `1.5`, `2.0`
- Battery: `12.0V`, `11.5V`
- Status: `"DEBUG: Testing"`, `"TEST MODE"`

## Condition Syntax

All conditions support:
- **Comparison operators**: `==`, `!=`, `>`, `<`, `>=`, `<=`
- **Logical operators**: `and`, `or`, `not`
- **String methods**: `.startswith()`, `.endswith()`, `.contains()`
- **Python expressions**: Any valid Python expression
- **Dot notation**: `msg.data`, `msg.field.subfield`

## Mutual Exclusivity

A bridge **cannot** have both `when` and `unless` clauses:

```robocon
// ❌ This will cause a validation error
Bridge[Topic] bad_bridge data : "topic" {
    when: "msg.data > 10"
    unless: "msg.data < 0"  // ERROR: Can't have both!
};
```

Choose one based on your use case:
- Use `when` if most messages should be **filtered out** (whitelist)
- Use `unless` if most messages should **pass through** (blacklist)

## Performance Notes

- Conditions are evaluated **before** transformations
- Failed conditions skip all processing (saves CPU)
- Evaluation errors are logged but don't crash the bridge
- Use simple conditions for best performance
