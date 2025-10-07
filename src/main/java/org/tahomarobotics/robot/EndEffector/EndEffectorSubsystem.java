package org.tahomarobotics.robot.EndEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

    public class EndEffectorSubsystem extends AbstractSubsystem {
    private TalonFX motor;
    public EndEffectorSubsystem() {
        Logger.info("End Effector Motor has been created");
        motor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);}

    @Override
    public void subsystemPeriodic() {

    }
}
