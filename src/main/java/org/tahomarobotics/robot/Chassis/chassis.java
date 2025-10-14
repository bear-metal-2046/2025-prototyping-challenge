package org.tahomarobotics.robot.Chassis;

public class chassis {

    public static final chassis INSTANCE = new chassis(); /* needs to be put into RobotContainer but I cant find where RobotContainer should go*/
    private final ChassisSubsystem chassis;

    //ChassisSubsystem instance

        public chassis() {
            this(new ChassisSubsystem());
        }
        public chassis(ChassisSubsystem chassis){
        this.chassis = chassis;
        }

}
