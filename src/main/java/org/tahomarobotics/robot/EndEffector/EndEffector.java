package org.tahomarobotics.robot.EndEffector;



import org.tinylog.Logger;

public class EndEffector implements AutoCloseable{
    private final EndEffectorSubsystem endEffector;

    public EndEffector() {
        this(new EndEffectorSubsystem());

    }

    EndEffector(EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffector = endEffectorSubsystem;
        Logger.info("Elevator initialized");
    }
    @Override
    public void close() throws Exception {

    }
}
