package org.tahomarobotics.robot.Chassis;

public class Chassis {

    public static final Chassis INSTANCE = new Chassis(); /* needs to be put into RobotContainer but I cant find where RobotContainer should go*/

    //ChassisSubsystem instance
    ChassisSubsystem chassis = ChassisSubsystem.INSTANCE;

        public Chassis(){



        }

}
