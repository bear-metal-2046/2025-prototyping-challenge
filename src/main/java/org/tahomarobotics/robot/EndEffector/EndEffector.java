package org.tahomarobotics.robot.EndEffector;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;

public class EndEffector {
    private static final EndEffector INSTANCE = new EndEffector();
    public static EndEffector getInstance(){return INSTANCE;}
    private EndEffector (){
        Motor.getConfigurator().apply(EndEffectorConstants.motorConfig);
    }


    private final TalonFX Motor = new TalonFX(RobotMap.ENDEFFECTOR);




}
