package org.tahomarobotics.robot.endeffector;



import org.tinylog.Logger;

public class EndEffector implements AutoCloseable{
    private final EndEffectorSubsystem endEffector;

    public EndEffector() {
        this(new EndEffectorSubsystem());

    }

    EndEffector(EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffector = endEffectorSubsystem;
        Logger.info("EndEffector initialized");
    }
    @Override
    public void close() throws Exception {

    }
}
