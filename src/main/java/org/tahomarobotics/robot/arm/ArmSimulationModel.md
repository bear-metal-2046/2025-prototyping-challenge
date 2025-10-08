# Differential Arm Simulation - Mathematical Model

## Overview
This document provides the detailed mathematical derivation for a 2-DOF differential arm mechanism driven by two DC motors. The system couples motor dynamics through a differential gearbox to control both elbow and wrist joints.

## Nomenclature

### Motor Variables
- $V_1, V_2$ - Applied voltages to motors 1 and 2 [V]
- $\theta_{m1}, \theta_{m2}$ - Motor shaft angles [rad]
- $\omega_{m1}, \omega_{m2}$ - Motor shaft angular velocities [rad/s]
- $\alpha_{m1}, \alpha_{m2}$ - Motor shaft angular accelerations [rad/s²]
- $i_{m1}, i_{m2}$ - Motor currents [A]
- $\tau_{m1}, \tau_{m2}$ - Motor output torques [N⋅m]

### Shaft Variables (after gearbox)
- $\theta_{s1}, \theta_{s2}$ - Input shaft angles [rad]
- $\omega_{s1}, \omega_{s2}$ - Input shaft angular velocities [rad/s]
- $\tau_{s1}, \tau_{s2}$ - Input shaft torques [N⋅m]

### Joint Variables
- $\theta_e$ - Elbow joint angle [rad]
- $\omega_e$ - Elbow joint angular velocity [rad/s]
- $\alpha_e$ - Elbow joint angular acceleration [rad/s²]
- $\theta_w$ - Wrist joint angle [rad]
- $\omega_w$ - Wrist joint angular velocity [rad/s]
- $\alpha_w$ - Wrist joint angular acceleration [rad/s²]

### Motor Parameters (Kraken X60)
- $K_t$ - Motor torque constant [N⋅m/A]
- $K_v$ - Motor velocity constant [rad/s/V]
- $R$ - Motor resistance [Ω]
- $J_m$ - Motor rotor moment of inertia [kg⋅m²]

### Mechanical Parameters
- $G$ - Gearbox ratio = 24:1
- $G_b$ - Bevel gear ratio = 45:15 = 3:1
- $\eta$ - Overall gear efficiency = 0.9
- $J_e$ - Elbow joint moment of inertia [kg⋅m²]
- $J_w$ - Wrist joint moment of inertia = 0.00156 kg⋅m²
- $m$ - End effector mass = 1.814 kg
- $L$ - Arm length = 0.4572 m
- $g$ - Gravitational acceleration = 9.81 m/s²

## DC Motor Model

For each motor, the electrical and mechanical equations are:

### Electrical Equation
$$V_i = R \cdot i_i + K_v \cdot \omega_{mi}$$

Solving for current:
$$i_i = \frac{V_i - K_v \cdot \omega_{mi}}{R}$$

### Mechanical Equation
$$\tau_{mi} = K_t \cdot i_i$$

### Motor Dynamics
$$J_m \cdot \alpha_{mi} = \tau_{mi} - \tau_{load,i}$$

where $\tau_{load,i}$ is the load torque reflected back to the motor.

## Gear Train

### Motor to Input Shaft
Two-stage reduction (60/10 × 48/12 = 24:1):

$$\theta_{si} = \frac{\theta_{mi}}{G}$$

$$\omega_{si} = \frac{\omega_{mi}}{G}$$

$$\tau_{si} = \tau_{mi} \cdot G \cdot \eta$$

## Differential Kinematics

### Forward Kinematics (Shaft → Joint)

The differential mechanism couples the input shafts to the joints:

**Elbow (sum mode):**
$$\theta_e = \frac{\theta_{s1} + \theta_{s2}}{2}$$

$$\omega_e = \frac{\omega_{s1} + \omega_{s2}}{2}$$

**Wrist (difference mode through bevel gears):**
$$\theta_w = \frac{\theta_{s1} - \theta_{s2}}{2} \cdot G_b = \frac{(\theta_{s1} - \theta_{s2}) \cdot 3}{2}$$

$$\omega_w = \frac{\omega_{s1} - \omega_{s2}}{2} \cdot G_b = \frac{(\omega_{s1} - \omega_{s2}) \cdot 3}{2}$$

### Inverse Kinematics (Joint → Shaft)

$$\theta_{s1} = \theta_e + \frac{\theta_w}{G_b} = \theta_e + \frac{\theta_w}{3}$$

$$\theta_{s2} = \theta_e - \frac{\theta_w}{G_b} = \theta_e - \frac{\theta_w}{3}$$

$$\omega_{s1} = \omega_e + \frac{\omega_w}{G_b} = \omega_e + \frac{\omega_w}{3}$$

$$\omega_{s2} = \omega_e - \frac{\omega_w}{G_b} = \omega_e - \frac{\omega_w}{3}$$

## Joint Dynamics

### Elbow Joint

The elbow experiences gravity torque due to the arm and end effector mass:

**Gravity torque:**
$$\tau_{g} = m \cdot g \cdot L \cdot \cos(\theta_e)$$

**Equation of motion:**
$$J_e \cdot \alpha_e = \tau_e - \tau_g$$

where $\tau_e$ is the applied torque to the elbow from the motors.

### Wrist Joint

The wrist has no external torques (no gravity component for rotation):

**Equation of motion:**
$$J_w \cdot \alpha_w = \tau_w$$

where $\tau_w$ is the applied torque to the wrist from the motors.

## Torque Coupling

The torques at the input shafts are related to the joint torques through the differential:

### Forward Torque Transform (Shaft → Joint)

From the differential kinematics, using virtual work or Jacobian transpose:

$$\tau_e = \frac{\tau_{s1} + \tau_{s2}}{2}$$

$$\tau_w = \frac{(\tau_{s1} - \tau_{s2}) \cdot G_b}{2} = \frac{(\tau_{s1} - \tau_{s2}) \cdot 3}{2}$$

### Inverse Torque Transform (Joint → Shaft)

The required shaft torques to produce desired joint torques:

$$\tau_{s1} = \tau_e + \frac{\tau_w}{G_b} = \tau_e + \frac{\tau_w}{3}$$

$$\tau_{s2} = \tau_e - \frac{\tau_w}{G_b} = \tau_e - \frac{\tau_w}{3}$$

## Reflected Inertia

The motor sees not only its own inertia but also the reflected inertia from the downstream components.

### Inertia Reflected to Motor Shaft

For motor $i$:
$$J_{m,total,i} = J_m + \frac{J_{reflected,i}}{G^2}$$

Where the reflected inertia depends on the differential coupling:

$$J_{reflected,1} = \frac{J_e}{4} + \frac{J_w \cdot G_b^2}{4} = \frac{J_e}{4} + \frac{9 J_w}{4}$$

$$J_{reflected,2} = \frac{J_e}{4} + \frac{J_w \cdot G_b^2}{4} = \frac{J_e}{4} + \frac{9 J_w}{4}$$

Note: Both motors see the same reflected inertia due to symmetric differential.

## Coupled Equations of Motion

### State Vector

$$\mathbf{x} = \begin{bmatrix} \theta_{m1} \\ \theta_{m2} \\ \omega_{m1} \\ \omega_{m2} \end{bmatrix}$$

### Input Vector

$$\mathbf{u} = \begin{bmatrix} V_1 \\ V_2 \end{bmatrix}$$

### State Space Form

$$\dot{\mathbf{x}} = \begin{bmatrix} \omega_{m1} \\ \omega_{m2} \\ \alpha_{m1} \\ \alpha_{m2} \end{bmatrix}$$

Where the accelerations are computed from:

1. Calculate motor currents from voltages and velocities
2. Calculate motor torques from currents
3. Calculate shaft torques (with gearing and efficiency)
4. Calculate joint velocities and accelerations from shaft states
5. Calculate gravity torque on elbow
6. Solve joint dynamics for required joint torques
7. Reflect joint torques back to shaft torques (load)
8. Reflect shaft torques back to motor torques (load)
9. Calculate motor accelerations from motor dynamics

## Simulation Algorithm

### Initialization
1. Set initial conditions: $\theta_e(0) = 90°$, $\theta_w(0) = 0°$
2. Calculate initial motor positions using inverse kinematics × gearing
3. Set initial velocities to zero

### Update Step (given $V_1$, $V_2$ at time $t$)

1. **Motor electrical dynamics:**
   - $i_1 = \frac{V_1 - K_v \omega_{m1}}{R}$
   - $i_2 = \frac{V_2 - K_v \omega_{m2}}{R}$

2. **Motor torques:**
   - $\tau_{m1} = K_t i_1$
   - $\tau_{m2} = K_t i_2$

3. **Shaft states (gearing):**
   - $\theta_{s1} = \theta_{m1} / G$, $\omega_{s1} = \omega_{m1} / G$
   - $\theta_{s2} = \theta_{m2} / G$, $\omega_{s2} = \omega_{m2} / G$
   - $\tau_{s1} = \tau_{m1} \cdot G \cdot \eta$
   - $\tau_{s2} = \tau_{m2} \cdot G \cdot \eta$

4. **Joint states (forward kinematics):**
   - $\theta_e = \frac{\theta_{s1} + \theta_{s2}}{2}$
   - $\omega_e = \frac{\omega_{s1} + \omega_{s2}}{2}$
   - $\theta_w = \frac{(\theta_{s1} - \theta_{s2}) \cdot 3}{2}$
   - $\omega_w = \frac{(\omega_{s1} - \omega_{s2}) \cdot 3}{2}$

5. **Joint torques (forward torque transform):**
   - $\tau_e = \frac{\tau_{s1} + \tau_{s2}}{2}$
   - $\tau_w = \frac{(\tau_{s1} - \tau_{s2}) \cdot 3}{2}$

6. **External torques:**
   - $\tau_g = m \cdot g \cdot L \cdot \cos(\theta_e)$

7. **Joint accelerations:**
   - $\alpha_e = \frac{\tau_e - \tau_g}{J_e}$
   - $\alpha_w = \frac{\tau_w}{J_w}$
   - Apply hard stops: if $\theta_e < -100°$ or $\theta_e > 100°$, set $\alpha_e = 0$, $\omega_e = 0$

8. **Shaft accelerations (inverse kinematics on accelerations):**
   - $\alpha_{s1} = \alpha_e + \frac{\alpha_w}{3}$
   - $\alpha_{s2} = \alpha_e - \frac{\alpha_w}{3}$

9. **Motor accelerations (through gearing):**
   - $\alpha_{m1} = \alpha_{s1} \cdot G$
   - $\alpha_{m2} = \alpha_{s2} \cdot G$

10. **Integrate (using RK4 or Euler):**
    - $\omega_{m1}(t+\Delta t) = \omega_{m1}(t) + \alpha_{m1} \cdot \Delta t$
    - $\omega_{m2}(t+\Delta t) = \omega_{m2}(t) + \alpha_{m2} \cdot \Delta t$
    - $\theta_{m1}(t+\Delta t) = \theta_{m1}(t) + \omega_{m1}(t) \cdot \Delta t$
    - $\theta_{m2}(t+\Delta t) = \theta_{m2}(t) + \omega_{m2}(t) \cdot \Delta t$

### Output Calculation

After integration, calculate all sensor outputs:

1. **Motor encoders (TalonFX):**
   - $\theta_{m1}$, $\omega_{m1}$
   - $\theta_{m2}$, $\omega_{m2}$

2. **Input shaft encoders (CANcoder 1 & 2):**
   - $\theta_{s1} = \theta_{m1} / G$, $\omega_{s1} = \omega_{m1} / G$
   - $\theta_{s2} = \theta_{m2} / G$, $\omega_{s2} = \omega_{m2} / G$

3. **Wrist encoder (CANcoder 3, via belt 52:15):**
   - $\theta_{w,encoder} = \theta_w \cdot \frac{52}{15}$
   - $\omega_{w,encoder} = \omega_w \cdot \frac{52}{15}$

4. **Joint states (internal, for control feedback):**
   - $\theta_e$, $\omega_e$
   - $\theta_w$, $\omega_w$

## Implementation Notes

1. **Units**: All calculations must be performed in SI units (radians, m, kg, N⋅m)
2. **Timestep**: Recommend $\Delta t = 0.001$ s (1 kHz) or synchronize with robot periodic rate (20ms typical)
3. **Numerical Integration**: Use RK4 for better accuracy, or Euler for simplicity
4. **Hard Stops**: Implement as velocity/acceleration clamps when position limits are reached
5. **Motor Parameters**: Obtain Kraken X60 parameters from vendor specifications or WPILib database

## Validation

To verify the model:
1. Test with equal voltages → only elbow should move
2. Test with opposite voltages → only wrist should move
3. Check gravity: horizontal arm ($\theta_e = 0$) should require holding torque
4. Check initial condition: arm should start at vertical ($\theta_e = 90°$)
5. Verify encoder ratios match mechanical design
