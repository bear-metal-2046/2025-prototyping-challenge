package org.tahomarobotics.robot.arm;

import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

public class Arm implements AutoCloseable {
    private final ArmSubsystem arm;

    public Arm() {
        Logger.info("Creating an instance of Arm...");
        arm = new Arm();
    }

    @Override
    public void close() throws Exception {
    }
}
