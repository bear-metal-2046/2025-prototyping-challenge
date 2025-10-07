package org.tahomarobotics.robot.EndEffector;



import org.tinylog.Logger;

public class EndEffector implements AutoCloseable{
    private final EndEffectorSubsystem endEffector;

    public EndEffector() {
        this(new EndEffectorSubsystem());

    }

    public EndEffector(EndEffectorSubsystem EndEffectorSubsystem) {
        this.endEffector = EndEffectorSubsystem;
        Logger.info("Elevator initialized");
    }
    @Override
    public void close() throws Exception {

    }
}
