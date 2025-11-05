package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

import static org.tahomarobotics.robot.endeffector.EndEffectorConstants.endEffectorMotorConfig;

public class EndEffectorSubsystem extends AbstractSubsystem {

        private final TalonFX endEffectorMotor;

        public EndEffectorSubsystem() {
            endEffectorMotor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);
            RobustConfigurator.tryConfigureTalonFX("End Effector Motor", endEffectorMotor, endEffectorMotorConfig);
        Logger.info("End Effector Motor has been created");
    }

    @Override
    public void subsystemPeriodic() {

    }
}
