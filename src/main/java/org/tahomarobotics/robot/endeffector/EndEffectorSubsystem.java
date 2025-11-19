package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Volt;

public class EndEffectorSubsystem extends AbstractSubsystem {

        private final TalonFX endEffectorMotor;

        public void Logging (){

        }

        public EndEffectorSubsystem() {
            endEffectorMotor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);
            RobustConfigurator.tryConfigureTalonFX("End Effector Motor",endEffectorMotor,EndEffectorConstants.endEffectorMotorConfig);
            Logger.recordOutput("EndEffector","End Effector Subsystem has been created");
       


        }


    public void setVoltage (Voltage voltage){
            endEffectorMotor.setVoltage(voltage.in(Volt));
            Logger.recordOutput("EndEffector/Voltage", (BooleanSupplier) endEffectorMotor.getMotorVoltage());
        }




    @Override
    public void subsystemPeriodic() {
        Logger.recordOutput("End Effector Current Voltage", (BooleanSupplier) endEffectorMotor.getMotorVoltage());

    }
}
