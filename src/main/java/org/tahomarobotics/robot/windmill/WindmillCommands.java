package org.tahomarobotics.robot.windmill;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class WindmillCommands {
    private static final Windmill windmill = new Windmill();
    //windmill instance can't be public but an initialization command can i think???
    public static final Command initialize = new Command() {
        @Override
        public void initialize() {
            windmill.initialize();
        }
    };
}
