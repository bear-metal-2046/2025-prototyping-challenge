

package org.tahomarobotics.robot;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.arm.ArmConstants;
import org.tahomarobotics.robot.arm.ArmSubsystem;
import org.tahomarobotics.robot.elevator.ElevatorConstants;
import org.tahomarobotics.robot.elevator.ElevatorSubsystem;



public class Windmill {

    private static WindmillPosition.TeamPositions currentPositions = WindmillPosition.TeamPositions.TEAM_POSITIONS;

    public static void setTeamPositions(WindmillPosition.TeamPositions positions) {
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
    public static Command low(ArmSubsystem arm, ElevatorSubsystem elevator) {
        WindmillPosition target = currentPositions.getLow();
        if (!validatePosition(target)) return Commands.none();
        return arm.runOnce(() -> {
            arm.setArmPosition(target.armAngle());
            elevator.moveToPosition(target.elevatorHeight());
        });
    }

    // Mid position command
    public static Command mid(ArmSubsystem arm, ElevatorSubsystem elevator) {
        WindmillPosition target = currentPositions.getMid();
        if (!validatePosition(target)) return Commands.none();
        return arm.runOnce(() -> {
            arm.setArmPosition(target.armAngle());
            elevator.moveToPosition(target.elevatorHeight());
        });
    }

    // High position command
    public static Command high(ArmSubsystem arm, ElevatorSubsystem elevator) {
        WindmillPosition target = currentPositions.getHigh();
        if (!validatePosition(target)) return Commands.none();
        return arm.runOnce(() -> {
            arm.setArmPosition(target.armAngle());
            elevator.moveToPosition(target.elevatorHeight());
        });
    }

    // Stow position command
    public static Command stow(ArmSubsystem arm, ElevatorSubsystem elevator) {
        WindmillPosition target = currentPositions.getStow();
        if (!validatePosition(target)) return Commands.none();
        return arm.runOnce(() -> {
            arm.setArmPosition(target.armAngle());
            elevator.moveToPosition(target.elevatorHeight());
        });
    }
}
