package org.tahomarobotics.robot.chassis;

public class Chassis {

    public static final Chassis INSTANCE = new Chassis();
    private final ChassisSubsystem chassis;


        public Chassis() {
            this(new ChassisSubsystem());
        }
        Chassis(ChassisSubsystem Chassis){
        this.chassis = Chassis;
        }

}
