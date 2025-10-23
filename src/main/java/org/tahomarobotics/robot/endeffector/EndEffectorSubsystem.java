package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

    public class EndEffectorSubsystem extends AbstractSubsystem {

        private final TalonFX endEffectorMotor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);

        public EndEffectorSubsystem() {
        Logger.info("End Effector Motor has been created");
    }

    @Override
    public void subsystemPeriodic() {

    }
}
