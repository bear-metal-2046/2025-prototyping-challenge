# EndEffector Subsystem Logging Guide

## Overview

The EndEffector subsystem implements comprehensive structured logging using both TinyLog (for console/file logging) and AdvantageKit Logger (for replay and AdvantageScope visualization). This guide explains how to enable, view, and configure EndEffector logging.

## Logging Architecture

The EndEffector subsystem uses a dual-logging approach:

1. **TinyLog**: For human-readable console/file logs with levels (trace, debug, info, warn, error)
2. **AdvantageKit Logger**: For structured data logging that can be replayed in AdvantageScope

## Log Categories

### 1. Lifecycle Events

Logged to both TinyLog and AdvantageKit:

- **Initialization**: When the subsystem is created
- **Shutdown**: When the subsystem is closed
- **Timestamps**: FPGA timestamp for each event
- **Simulation Flag**: Whether running in simulation mode

**AdvantageScope Paths:**
```
EndEffector/Lifecycle/Event
EndEffector/Lifecycle/Message
EndEffector/Lifecycle/Timestamp
EndEffector/Lifecycle/IsSimulation
```

**TinyLog Examples:**
```
[INFO] EndEffectorSubsystem initialized - Motor ID: 10, Sim: false
[INFO] EndEffectorSubsystem shutdown
```

### 2. Command Execution

Logged when commands start, complete, or are interrupted:

- Command name
- Start/end timestamps
- Interruption status

**AdvantageScope Paths:**
```
EndEffector/Command/Name
EndEffector/Command/StartTime
EndEffector/Command/EndTime
EndEffector/Command/Interrupted
```

**TinyLog Examples:**
```
[DEBUG] EndEffector command starting: applyVoltage
[DEBUG] EndEffector command completed: applyVoltage
[DEBUG] EndEffector command interrupted: applyVoltage
```

### 3. Actuator Output Deltas

Logged when motor voltage changes significantly (configurable threshold):

- Previous voltage value
- New voltage value
- Delta magnitude
- Timestamp

**AdvantageScope Paths:**
```
EndEffector/ActuatorDelta/PreviousVoltage
EndEffector/ActuatorDelta/NewVoltage
EndEffector/ActuatorDelta/Delta
EndEffector/ActuatorDelta/Timestamp
```

**TinyLog Examples:**
```
[DEBUG] EndEffector voltage change: 0.00V -> 3.50V (delta: 3.50V)
```

### 4. Periodic Telemetry

Logged every robot loop (20ms) to AdvantageKit, with rate-limited TinyLog:

**Motor Status:**
- Voltage
- Stator current
- Supply current
- Temperature
- Position
- Velocity

**AdvantageScope Paths:**
```
EndEffector/Voltages/Motor
EndEffector/Currents/Stator
EndEffector/Currents/Supply
EndEffector/Temperature/Motor
EndEffector/Position
EndEffector/Velocity
```

**TinyLog Examples (rate-limited):**
```
[TRACE] EndEffector periodic: voltage=3.50V, current=2.15A, temp=28.3C
```

## Configuration

### Logging Constants

Located in `EndEffectorConstants.java`:

```java
/** Minimum time between periodic logging snapshots (seconds) */
public static final double PERIODIC_LOG_INTERVAL = 0.5;

/** Minimum voltage change to trigger actuator delta logging (volts) */
public static final double VOLTAGE_DELTA_THRESHOLD = 0.5;

/** Minimum time between rate-limited logs of the same type (seconds) */
public static final double RATE_LIMIT_INTERVAL = 1.0;
```

### TinyLog Configuration

Edit `src/main/resources/tinylog.properties`:

```properties
# Set global log level (trace, debug, info, warn, error)
level=info

# Enable debug logging for EndEffector only
level@org.tahomarobotics.robot.endeffector=debug

# Enable trace logging for all subsystems (very verbose)
level@org.tahomarobotics.robot=trace
```

### Rate Limiting

Rate limiting prevents log flooding in high-frequency operations:

- **Periodic snapshots**: Limited by `PERIODIC_LOG_INTERVAL` (default: 0.5s)
- **Voltage deltas**: Limited by `RATE_LIMIT_INTERVAL` (default: 1.0s)
- Only significant changes (above `VOLTAGE_DELTA_THRESHOLD`) are logged

## Viewing Logs

### Console Output

Logs appear in the console when running the robot code:

```bash
./gradlew simulateJava
```

Look for lines prefixed with the log level and class name:
```
12:34:56.789 INFO EndEffectorSubsystem - EndEffectorSubsystem initialized - Motor ID: 10, Sim: true
```

### AdvantageScope

1. **Start the robot** (real or simulation)
2. **Open AdvantageScope**
3. **Connect to robot:**
   - Press `Ctrl+Shift+K` (or menu: File → Connect to Robot)
   - For simulation: Use team address `10.20.46.2`
   - For real robot: Use team number `2046`

4. **View EndEffector data:**
   - In the left sidebar, expand the `EndEffector` node
   - Add signals to plots by dragging them to the graph area
   - Use Line Graph for continuous data (voltage, current, temperature)
   - Use Discrete Legend for state changes (lifecycle events, commands)

5. **Common visualizations:**
   - **Motor Performance**: Plot `EndEffector/Voltages/Motor` and `EndEffector/Currents/Stator`
   - **Temperature Monitoring**: Plot `EndEffector/Temperature/Motor`
   - **Command Timeline**: Show `EndEffector/Command/Name` with `EndEffector/Command/Interrupted`
   - **Voltage Changes**: Plot `EndEffector/ActuatorDelta/Delta`

### Log Files

If file logging is enabled in `tinylog.properties`:

```properties
writer=file
writer.file=/home/lvuser/logs/robot.log
```

Logs are written to the specified file on the RoboRIO.

## Replay and Analysis

### Recording Logs

AdvantageKit automatically records all `Logger.recordOutput()` calls. To save a log:

1. Run the robot
2. In AdvantageScope, click "Start Recording"
3. Perform operations
4. Click "Stop Recording"
5. Save the `.wpilog` file

### Replaying Logs

1. Open AdvantageScope
2. File → Open Log File
3. Select your `.wpilog` file
4. Scrub through time using the timeline slider
5. All EndEffector telemetry is available for analysis

### Debugging with Replays

**Example: Investigating a voltage spike**

1. Load the replay log
2. Plot `EndEffector/Voltages/Motor`
3. Find the spike in the timeline
4. Check `EndEffector/ActuatorDelta/Delta` at that timestamp
5. View `EndEffector/Command/Name` to see what command was running
6. Cross-reference with `EndEffector/Currents/Stator` to see if current spiked

## Troubleshooting

### No Logs Appearing in Console

**Check TinyLog configuration:**
```properties
# Ensure level is not set too high
level=info
level@org.tahomarobotics.robot.endeffector=debug
```

**Verify writer is enabled:**
```properties
writer=console
```

### No Data in AdvantageScope

**Check connection:**
- Verify robot is running
- Correct team number or IP address
- NetworkTables is enabled

**Verify Logger is started:**
In `Robot.java`:
```java
Logger.addDataReceiver(new NT4Publisher());
Logger.start();
```

### Too Many Logs (Performance Impact)

**Increase rate limit intervals in `EndEffectorConstants.java`:**
```java
public static final double PERIODIC_LOG_INTERVAL = 2.0;  // More infrequent
public static final double RATE_LIMIT_INTERVAL = 5.0;    // Less frequent deltas
```

**Reduce TinyLog level for competition:**
```properties
level=warn  # Only warnings and errors
```

### Missing Voltage Delta Logs

**Lower the threshold in `EndEffectorConstants.java`:**
```java
public static final double VOLTAGE_DELTA_THRESHOLD = 0.1;  // More sensitive
```

## Best Practices

1. **Development**: Use `debug` or `trace` level for detailed diagnostics
2. **Competition**: Use `info` level to reduce overhead
3. **Testing**: Enable AdvantageScope recording for all test runs
4. **Debugging**: Review replay logs instead of adding print statements
5. **Performance**: Keep rate limit intervals reasonable (≥0.5s for periodic logs)
6. **Thresholds**: Adjust `VOLTAGE_DELTA_THRESHOLD` based on your use case

## Integration with Other Systems

### CommandLogger

The EndEffector integrates with the global `CommandLogger` utility, which automatically logs all command start/finish/interrupt events with timing information.

### ExecutionTimeLogger

The `subsystemPeriodic()` method is automatically wrapped by `ExecutionTimeLogger` via `AbstractSubsystem`, logging execution time under `ExecTimes/EndEffectorSubsystem`.

## Example Logging Session

```
12:34:56.123 INFO  EndEffectorSubsystem - EndEffectorSubsystem initialized - Motor ID: 10, Sim: true
12:34:56.124 INFO  EndEffector - EndEffector initialized - Sim: true
12:34:57.001 DEBUG EndEffector - EndEffector command starting: applyVoltage
12:34:57.003 DEBUG EndEffectorSubsystem - EndEffector voltage change: 0.00V -> 6.00V (delta: 6.00V)
12:34:58.503 TRACE EndEffectorSubsystem - EndEffector periodic: voltage=6.00V, current=3.42A, temp=29.1C
12:35:00.250 DEBUG EndEffector - EndEffector command completed: applyVoltage
12:35:10.999 INFO  EndEffector - EndEffector closing
12:35:11.000 INFO  EndEffectorSubsystem - EndEffectorSubsystem shutdown
```

## Additional Resources

- [TinyLog Documentation](https://tinylog.org/v2/)
- [AdvantageKit Documentation](https://docs.advantagekit.org/)
- [AdvantageScope Documentation](https://docs.advantagescope.org/)
- [ADVANTAGEKIT.md](./ADVANTAGEKIT.md) - Project-specific AdvantageKit guide
- [TINYLOG.md](./TINYLOG.md) - Project-specific TinyLog guide
