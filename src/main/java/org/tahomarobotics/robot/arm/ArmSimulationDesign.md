# Arm Simulation Design

## Overview
This document describes the simulation model for a differential arm mechanism with elbow and wrist joints driven by two motors in a differential configuration.

## Mechanical System

### Joint Structure
- **Elbow Joint**: Primary arm rotation with gravity effects (modeled using cosine function)
- **Wrist Joint**: Secondary rotation for end effector positioning

### Motor Configuration
- **Motor 1**: Drives differential input shaft 1
- **Motor 2**: Drives differential input shaft 2
- **Differential Behavior**:
  - Same direction rotation → Elbow joint moves
  - Opposite direction rotation → Wrist joint moves

## Gear Train

### Motor to Differential Input Shafts
Two-stage gearbox per motor:
1. **Stage 1**: 60T gear / 10T pinion = 6:1 reduction
2. **Stage 2**: 48T sprocket / 12T sprocket (chain) = 4:1 reduction
3. **Total reduction**: 24:1 per motor to input shaft

### Differential Gearing (Bevel Gears)
Configuration: Ring and planet bevel gear differential
- **Input small bevel gears (planets)**: 15 teeth each
  - Horizontally oriented on opposite sides of ring gear
  - Input shafts are coaxial with opposite engagement
- **Output large bevel gear (ring)**: 45 teeth
  - Connected to wrist shaft
- **Bevel gear ratio**: 45:15 = 3:1

### Encoder Belt Drive
- **Wrist shaft pulley**: 52 teeth
- **Encoder pulley**: 15 teeth
- **Ratio**: 52:15 ≈ 3.467:1

## Simulation Components

### Physics Model
**Custom 2-DOF Differential Arm Simulator**

Due to the differential coupling between motors and joints, a custom unified model is required rather than separate `SingleJointedArmSim` and `DCMotorSim` instances. The standard WPILib simulation classes only accept voltage inputs and cannot handle external torques or coupled dynamics.

**Inputs:**
- Voltage to Motor 1 (V1)
- Voltage to Motor 2 (V2)

**Outputs:**
- Motor 1: position, velocity (for TalonFX sim)
- Motor 2: position, velocity (for TalonFX sim)
- Elbow joint: position, velocity
- Wrist joint: position, velocity
- Input shaft 1: position, velocity (for CANcoder sim)
- Input shaft 2: position, velocity (for CANcoder sim)
- Wrist shaft: position, velocity (for CANcoder sim via belt)

**Model includes:**
- DC motor dynamics for both Kraken X60 motors
- Differential kinematics (forward and coupling)
- Elbow gravity torque (cosine-based)
- Gear train reductions and efficiency
- Hard stops for elbow joint
- Moment of inertia for all rotating components

See `ArmSimulationModel.md` for detailed mathematical derivation.

### Motor Simulations
- **Motor 1 Sim** (Kraken X60 with TalonFX)
  - Receives: Applied voltage
  - Provides: Rotational position, rotational speed

- **Motor 2 Sim** (Kraken X60 with TalonFX)
  - Receives: Applied voltage
  - Provides: Rotational position, rotational speed

### Encoder Simulations
Three CANcoder absolute encoders:
1. **Input Shaft 1 Encoder**: Direct coupled to input shaft 1 (small bevel gear)
2. **Input Shaft 2 Encoder**: Direct coupled to input shaft 2 (small bevel gear)
3. **Wrist Encoder**: Connected to wrist shaft via belt (52:15 ratio)

Each encoder receives:
- Rotational position
- Rotational velocity

## Differential Kinematics

### Forward Kinematics (Motor Positions → Joint Positions)

Let:
- `θ_m1` = Motor 1 shaft angle
- `θ_m2` = Motor 2 shaft angle
- `θ_elbow` = Elbow joint angle
- `θ_wrist` = Wrist joint angle

After gearbox (24:1 reduction):
- `θ_s1 = θ_m1 / 24` (input shaft 1)
- `θ_s2 = θ_m2 / 24` (input shaft 2)

Differential equations (with k_e = k_w = 1.0):
- **Elbow**: `θ_elbow = (θ_s1 + θ_s2) / 2`
- **Wrist**: `θ_wrist = (θ_s1 - θ_s2) / 2 * (45/15) = (θ_s1 - θ_s2) * 1.5`

### Inverse Kinematics (Joint Positions → Motor Positions)

To achieve desired elbow and wrist positions:
- `θ_s1 = θ_elbow + (θ_wrist * (15/45)) = θ_elbow + (θ_wrist / 3)`
- `θ_s2 = θ_elbow - (θ_wrist * (15/45)) = θ_elbow - (θ_wrist / 3)`
- `θ_m1 = θ_s1 * 24`
- `θ_m2 = θ_s2 * 24`

## Hard Stops

### Elbow Joint Limits
- Minimum angle: -100 degrees
- Maximum angle: +100 degrees
- Total range: 200 degrees

### Wrist Joint Limits
- **No hard stops** (continuous rotation enabled by slip rings for end-effector wiring)

## Initial Conditions

- **Elbow angle**: 90 degrees
- **Wrist angle**: 0 degrees

## Physical Parameters

### Gear Efficiency
- **Overall efficiency**: 90% (0.9)
- Applied across the gear train

### Elbow (SingleJointedArmSim)
- **Arm length**: 18 inches (0.4572 m)
- **End effector mass**: 4 lbs (1.814 kg)
- These values are preliminary and will be refined as mechanical design progresses
- **NOTE**: All parameters must use WPILib unit variables for proper metric conversion

### Wrist (DCMotorSim)
- **Rotational inertia**: 0.548 lb·in² (0.00156 kg·m²)
- Preliminary value, subject to refinement

### Sign Conventions

#### Elbow Joint
- **+90 degrees**: Arm vertical (pointing up)
- **0 degrees**: Arm horizontal
- **-90 degrees**: Arm vertical (pointing down)
- **Range**: -100° to +100°

#### Wrist Joint
- **Right-hand rule**: Thumb points outward along rotation axis (away from elbow)
- **Positive rotation**: Counter-clockwise when viewed from end effector
- **0 degrees**: Reference position (to be defined based on end effector orientation)
- **No limits**: Continuous rotation

## Implementation Notes

1. **Unit Management**: All physical parameters must be defined using WPILib Units classes
   - Distances: `edu.wpi.first.units.Units.Inches` / `Units.Meters`
   - Mass: `Units.Pounds` / `Units.Kilograms`
   - Inertia: Convert lb·in² to kg·m² using unit conversions
   - Angles: `Units.Degrees` / `Units.Radians`

2. **Parameter Refinement**: Physical parameters are preliminary estimates that will be updated as the mechanical design is finalized

## Simulation Flow

```
     V1, V2 (motor voltages)
            │
            v
    ┌───────────────────────┐
    │   2-DOF Differential  │
    │   Arm Simulator       │
    │                       │
    │ - Motor dynamics      │
    │ - Gear train (24:1)   │
    │ - Differential        │
    │ - Joint dynamics      │
    │ - Gravity (elbow)     │
    │ - Hard stops (elbow)  │
    └───────────┬───────────┘
                │
                v
    ┌───────────────────────────────┐
    │ Outputs:                      │
    │ - θ_m1, ω_m1 → TalonFX 1     │
    │ - θ_m2, ω_m2 → TalonFX 2     │
    │ - θ_s1, ω_s1 → CANcoder 1    │
    │ - θ_s2, ω_s2 → CANcoder 2    │
    │ - θ_wrist, ω_wrist (via belt) │
    │   → CANcoder 3                │
    │ - θ_elbow, ω_elbow (internal) │
    └───────────────────────────────┘
```

## Next Steps

1. Clarify mechanical specifications (questions above)
2. Define coordinate systems and sign conventions
3. Implement differential kinematics equations
4. Integrate SingleJointedArmSim and DCMotorSim
5. Add hard stop logic
6. Implement encoder simulations with belt ratio
7. Add unit tests for kinematics
8. Validate against physical system (if available)
