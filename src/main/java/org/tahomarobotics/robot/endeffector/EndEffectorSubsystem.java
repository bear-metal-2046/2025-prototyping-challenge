package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tinylog.Logger;

import static edu.wpi.first.units.Units.Volt;

public class EndEffectorSubsystem extends AbstractSubsystem {

        private final TalonFX endEffectorMotor;

        public EndEffectorSubsystem() {
            endEffectorMotor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);
            RobustConfigurator.tryConfigureTalonFX("End Effector Motor",endEffectorMotor,EndEffectorConstants.endEffectorMotorConfig);
        Logger.info("End Effector Motor has been created");

    }
    public void setVoltage (Voltage voltage){
            endEffectorMotor.setVoltage(voltage.in(Volt));
        }

    @Override
    public void subsystemPeriodic() {

    }
}
