package org.tahomarobotics.robot.EndEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

    public class EndEffectorSubsystem extends AbstractSubsystem {

    public EndEffectorSubsystem() {
        Logger.info("End Effector Motor has been created");
       final TalonFX EndEffectorMotor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);
        RobustConfigurator.tryConfigureTalonFX("End Effector Motor", EndEffectorMotor,EndEffectorConstants.EndEffectorMotorConfig);}


    @Override
    public void subsystemPeriodic() {

    }
}
