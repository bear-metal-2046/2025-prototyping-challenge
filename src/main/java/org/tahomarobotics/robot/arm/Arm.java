package org.tahomarobotics.robot.arm;

import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

public class Arm implements AutoCloseable {
    private final ArmSubsystem subsystem;

    public Arm(ArmSubsystem subsystem) {
        Logger.info("Creating an instance of Arm...");
        this.subsystem = subsystem;
    }

    @Override
    public void close() throws Exception {
    }
}
