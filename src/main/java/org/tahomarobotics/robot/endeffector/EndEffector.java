package org.tahomarobotics.robot.endeffector;



import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import org.tinylog.Logger;
import org.tinylog.Supplier;

public class EndEffector implements AutoCloseable{
    private final EndEffectorSubsystem endEffector;

    public EndEffector() {
        this(new EndEffectorSubsystem());


    }

    EndEffector(EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffector = endEffectorSubsystem;
        Logger.info("End Effector has been intialized");
    }
    public Command applyVoltage (Supplier <Voltage> voltage) {
        return endEffector.run(() -> endEffector.setVoltage(voltage.get()));
    }
    public void setDefaultCommand (Command command){
        endEffector.setDefaultCommand(command);

    }

    @Override
    public void close() throws Exception {


    }
}
