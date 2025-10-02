package org.tahomarobotics.robot;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.util.AbstractSubsystem;
/* From what I've heard from Rod, this class should also implement something,
   which I think might be like "subsystemPeriodic()" after "AbstractSubsystem" */
public class EndEffectorSubsystem extends AbstractSubsystem {
    /*
    to-do list:
    PRIORITIES - Position Control, Velocity Control, Calibration
    (Calibration is a fancy word for Zeroing)

    Use 1 TalonFX motor
    focus on pos & vel control before calibration
    */




}
