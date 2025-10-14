package org.tahomarobotics.robot.chassis;

public class Chassis {

    public static final Chassis INSTANCE = new Chassis(); /* needs to be put into RobotContainer but I cant find where RobotContainer should go*/
    private final ChassisSubsystem Chassis;

    //ChassisSubsystem instance

        public Chassis() {
            this(new ChassisSubsystem());
        }
        public Chassis(ChassisSubsystem Chassis){
        this.Chassis = Chassis;
        }

}
