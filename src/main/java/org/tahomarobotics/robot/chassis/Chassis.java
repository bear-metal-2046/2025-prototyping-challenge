package org.tahomarobotics.robot.chassis;

public class Chassis {


    private final ChassisSubsystem chassis;


        public Chassis() {
            this(new ChassisSubsystem());
        }
        Chassis(ChassisSubsystem Chassis){
        this.chassis = Chassis;
        }

}
