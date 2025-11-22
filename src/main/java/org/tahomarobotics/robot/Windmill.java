

package org.tahomarobotics.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.arm.Arm;
import org.tahomarobotics.robot.arm.ArmConstants;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;


public class Windmill {

    public Windmill(Arm arm , Elevator elevator) {
    }


    private  WindmillPosition.TeamPositions currentPositions = WindmillPosition.TeamPositions.TEAM_POSITIONS;

    public void setTeamPositions(WindmillPosition.TeamPositions positions) {
        currentPositions = positions;
    }

    private static boolean validatePosition(WindmillPosition pos) {
        // Convert to primitives
        double angleDegrees = pos.armAngle().in(Units.Degrees);
        double heightInches = pos.elevatorHeight().in(Units.Inches);

        boolean armValid = angleDegrees >= ArmConstants.MIN_POSITION.in(Units.Degrees) && angleDegrees <= ArmConstants.MAX_POSITION.in(Units.Degrees);
        boolean elevatorValid = heightInches >= ElevatorConstants.ELEVATOR_MIN_POSE.in(Units.Inches) && heightInches <= ElevatorConstants.ELEVATOR_MAX_POSE.in(Units.Inches);

        return armValid && elevatorValid;
    }

    // Low position command
    public Command moveWindmill(Arm arm, Elevator elevator, WindmillPosition height) {

        if (!validatePosition(height)) return Commands.none();
        return (
            arm.setArmPosition(height.armAngle())
                    .alongWith(elevator.setPosition(height.elevatorHeight()))

        );
    }


}
