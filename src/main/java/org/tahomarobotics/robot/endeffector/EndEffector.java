package org.tahomarobotics.robot.endeffector;



import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.tinylog.Supplier;

public class EndEffector implements AutoCloseable {
    private final EndEffectorSubsystem endEffector;

    public EndEffector() {
        this(new EndEffectorSubsystem());
    }

    EndEffector(EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffector = endEffectorSubsystem;
        
        // Structured lifecycle logging
        org.tinylog.Logger.info("EndEffector initialized - Sim: {}", RobotBase.isSimulation());
        Logger.recordOutput("EndEffector/Initialized", true);
        Logger.recordOutput("EndEffector/InitTimestamp", Timer.getFPGATimestamp());
    }
    
    public Command applyVoltage(Supplier<Voltage> voltage) {
        return endEffector.run(() -> endEffector.setVoltage(voltage.get()))
            .beforeStarting(() -> {
                org.tinylog.Logger.debug("EndEffector command starting: applyVoltage");
                Logger.recordOutput("EndEffector/Command/Name", "applyVoltage");
                Logger.recordOutput("EndEffector/Command/StartTime", Timer.getFPGATimestamp());
            })
            .finallyDo((interrupted) -> {
                if (interrupted) {
                    org.tinylog.Logger.debug("EndEffector command interrupted: applyVoltage");
                    Logger.recordOutput("EndEffector/Command/Interrupted", true);
                } else {
                    org.tinylog.Logger.debug("EndEffector command completed: applyVoltage");
                    Logger.recordOutput("EndEffector/Command/Interrupted", false);
                }
                Logger.recordOutput("EndEffector/Command/EndTime", Timer.getFPGATimestamp());
            });
    }
    
    public void setDefaultCommand(Command command) {
        endEffector.setDefaultCommand(command);
        org.tinylog.Logger.info("EndEffector default command set: {}", command.getName());
        Logger.recordOutput("EndEffector/DefaultCommand", command.getName());
    }
    
    @Override
    public void close() throws Exception {
        org.tinylog.Logger.info("EndEffector closing");
        Logger.recordOutput("EndEffector/Closing", true);
        Logger.recordOutput("EndEffector/CloseTimestamp", Timer.getFPGATimestamp());
        endEffector.close();
    }
}
