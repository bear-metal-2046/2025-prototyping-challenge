package org.tahomarobotics.robot.arm;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

class ArmDifferentialTransformTest {

    private static final double TOLERANCE = 1e-9;

    @Test
    @DisplayName("When both motors move same direction at same speed, only elbow mode is active")
    void bothMotorsSameDirection_OnlyElbowMode() {
        // Both motors rotating forward at 1 rad/s
        DiffMotorVelocities motorVels = new DiffMotorVelocities(
            RadiansPerSecond.of(1.0),
            RadiansPerSecond.of(1.0)
        );

        VirtualMotorVelocities result = ArmDifferentialTransform.transform(motorVels);

        // Sum mode: (1 + 1) / 2 = 1
        // Difference mode: (1 - 1) / 2 = 0
        assertEquals(1.0, result.elbowAngularVelocity().in(RadiansPerSecond), TOLERANCE,
            "Elbow velocity should be (top + bottom) / 2");
        assertEquals(0.0, result.wristAngularVelocity().in(RadiansPerSecond), TOLERANCE,
            "Wrist velocity should be zero when motors move together");
    }

    @Test
    @DisplayName("When motors move opposite directions at same speed, only wrist mode is active")
    void motorsOppositeDirection_OnlyWristMode() {
        // Top motor forward, bottom motor backward at 1 rad/s
        DiffMotorVelocities motorVels = new DiffMotorVelocities(
            RadiansPerSecond.of(1.0),
            RadiansPerSecond.of(-1.0)
        );

        VirtualMotorVelocities result = ArmDifferentialTransform.transform(motorVels);

        // Sum mode: (1 + (-1)) / 2 = 0
        // Difference mode: (1 - (-1)) / 2 = 1
        assertEquals(0.0, result.elbowAngularVelocity().in(RadiansPerSecond), TOLERANCE,
            "Elbow velocity should be zero when motors oppose");
        assertEquals(1.0, result.wristAngularVelocity().in(RadiansPerSecond), TOLERANCE,
            "Wrist velocity should be (top - bottom) / 2");
    }

    @Test
    @DisplayName("Forward then inverse velocity transform should be identity")
    void velocityTransform_RoundTrip() {
        // Start with arbitrary motor velocities
        DiffMotorVelocities original = new DiffMotorVelocities(
            RadiansPerSecond.of(3.5),
            RadiansPerSecond.of(1.2)
        );

        // Forward: motor -> virtual
        VirtualMotorVelocities virtual = ArmDifferentialTransform.transform(original);

        // Inverse: virtual -> motor
        DiffMotorVelocities recovered = ArmDifferentialTransform.transform(virtual);

        assertEquals(original.topMotorAngularVelocity().in(RadiansPerSecond),
                     recovered.topMotorAngularVelocity().in(RadiansPerSecond),
                     TOLERANCE, "Top motor velocity should survive round-trip");
        assertEquals(original.bottomMotorAngularVelocity().in(RadiansPerSecond),
                     recovered.bottomMotorAngularVelocity().in(RadiansPerSecond),
                     TOLERANCE, "Bottom motor velocity should survive round-trip");
    }

    @Test
    @DisplayName("Forward then inverse position transform should be identity")
    void positionTransform_RoundTrip() {
        // Start with arbitrary motor positions
        DiffMotorPositions original = new DiffMotorPositions(
            Radians.of(2.7),
            Radians.of(-1.3)
        );

        // Forward: motor -> virtual
        VirtualMotorPositions virtual = ArmDifferentialTransform.transform(original);

        // Inverse: virtual -> motor
        DiffMotorPositions recovered = ArmDifferentialTransform.transform(virtual);

        assertEquals(original.topMotorPosition().in(Radians),
                     recovered.topMotorPosition().in(Radians),
                     TOLERANCE, "Top motor position should survive round-trip");
        assertEquals(original.bottomMotorPosition().in(Radians),
                     recovered.bottomMotorPosition().in(Radians),
                     TOLERANCE, "Bottom motor position should survive round-trip");
    }

    @Test
    @DisplayName("Inverse then forward velocity transform should be identity")
    void velocityTransform_ReverseRoundTrip() {
        // Start with virtual motor velocities
        VirtualMotorVelocities original = new VirtualMotorVelocities(
            RadiansPerSecond.of(0.5),
            RadiansPerSecond.of(0.3)
        );

        // Inverse: virtual -> motor
        DiffMotorVelocities motor = ArmDifferentialTransform.transform(original);

        // Forward: motor -> virtual
        VirtualMotorVelocities recovered = ArmDifferentialTransform.transform(motor);

        assertEquals(original.elbowAngularVelocity().in(RadiansPerSecond),
                     recovered.elbowAngularVelocity().in(RadiansPerSecond),
                     TOLERANCE, "Elbow velocity should survive round-trip");
        assertEquals(original.wristAngularVelocity().in(RadiansPerSecond),
                     recovered.wristAngularVelocity().in(RadiansPerSecond),
                     TOLERANCE, "Wrist velocity should survive round-trip");
    }

    @Test
    @DisplayName("Inverse then forward position transform should be identity")
    void positionTransform_ReverseRoundTrip() {
        // Start with virtual motor positions
        VirtualMotorPositions original = new VirtualMotorPositions(
            Radians.of(1.2),
            Radians.of(-0.8)
        );

        // Inverse: virtual -> motor
        DiffMotorPositions motor = ArmDifferentialTransform.transform(original);

        // Forward: motor -> virtual
        VirtualMotorPositions recovered = ArmDifferentialTransform.transform(motor);

        assertEquals(original.elbowMotorPosition().in(Radians),
                     recovered.elbowMotorPosition().in(Radians),
                     TOLERANCE, "Elbow position should survive round-trip");
        assertEquals(original.wristMotorPosition().in(Radians),
                     recovered.wristMotorPosition().in(Radians),
                     TOLERANCE, "Wrist position should survive round-trip");
    }

    @Test
    @DisplayName("Pure elbow motion: equal motor positions")
    void pureElbowMotion_EqualMotors() {
        // Elbow at 45°, wrist at 0°
        VirtualMotorPositions virtual = new VirtualMotorPositions(
            Radians.of(Math.toRadians(45.0)),
            Radians.of(0.0)
        );

        DiffMotorPositions motor = ArmDifferentialTransform.transform(virtual);

        // elbow + wrist = 45 + 0 = 45
        // elbow - wrist = 45 - 0 = 45
        assertEquals(Math.toRadians(45.0), motor.topMotorPosition().in(Radians), TOLERANCE,
            "Top motor should equal elbow position");
        assertEquals(Math.toRadians(45.0), motor.bottomMotorPosition().in(Radians), TOLERANCE,
            "Bottom motor should equal elbow position");
    }

    @Test
    @DisplayName("Pure wrist motion: opposite motor positions")
    void pureWristMotion_OppositeMotors() {
        // Elbow at 0°, wrist at 30°
        VirtualMotorPositions virtual = new VirtualMotorPositions(
            Radians.of(0.0),
            Radians.of(Math.toRadians(30.0))
        );

        DiffMotorPositions motor = ArmDifferentialTransform.transform(virtual);

        // top = 0 + 30 = 30
        // bottom = 0 - 30 = -30
        assertEquals(Math.toRadians(30.0), motor.topMotorPosition().in(Radians), TOLERANCE,
            "Top motor should be positive");
        assertEquals(Math.toRadians(-30.0), motor.bottomMotorPosition().in(Radians), TOLERANCE,
            "Bottom motor should be negative (opposite)");
    }

    @Test
    @DisplayName("Combined motion: elbow and wrist additive")
    void combinedMotion_ElbowAndWrist() {
        // Elbow at 90°, wrist at 45°
        VirtualMotorPositions virtual = new VirtualMotorPositions(
            Radians.of(Math.toRadians(90.0)),
            Radians.of(Math.toRadians(45.0))
        );

        DiffMotorPositions motor = ArmDifferentialTransform.transform(virtual);

        // top = 90 + 45 = 135
        // bottom = 90 - 45 = 45
        assertEquals(Math.toRadians(135.0), motor.topMotorPosition().in(Radians), TOLERANCE,
            "Top motor should be elbow + wrist");
        assertEquals(Math.toRadians(45.0), motor.bottomMotorPosition().in(Radians), TOLERANCE,
            "Bottom motor should be elbow - wrist");
    }

    @Test
    @DisplayName("Differential sum and difference are inverses")
    void sumAndDifference_AreInverses() {
        double top = 5.0;
        double bottom = 3.0;

        // Apply sum/difference
        double sum = (top + bottom) / 2.0;      // = 4.0
        double diff = (top - bottom) / 2.0;     // = 1.0

        // Reverse: recover top and bottom
        double recoveredTop = sum + diff;       // = 5.0
        double recoveredBottom = sum - diff;    // = 3.0

        assertEquals(top, recoveredTop, TOLERANCE, "Should recover top from sum + diff");
        assertEquals(bottom, recoveredBottom, TOLERANCE, "Should recover bottom from sum - diff");
    }

    @Test
    @DisplayName("Zero velocities transform to zero velocities")
    void zeroVelocities_StayZero() {
        DiffMotorVelocities zero = new DiffMotorVelocities(
            RadiansPerSecond.of(0.0),
            RadiansPerSecond.of(0.0)
        );

        VirtualMotorVelocities result = ArmDifferentialTransform.transform(zero);

        assertEquals(0.0, result.elbowAngularVelocity().in(RadiansPerSecond), TOLERANCE);
        assertEquals(0.0, result.wristAngularVelocity().in(RadiansPerSecond), TOLERANCE);
    }

    @Test
    @DisplayName("Zero positions transform to zero positions")
    void zeroPositions_StayZero() {
        DiffMotorPositions zero = new DiffMotorPositions(
            Radians.of(0.0),
            Radians.of(0.0)
        );

        VirtualMotorPositions result = ArmDifferentialTransform.transform(zero);

        assertEquals(0.0, result.elbowMotorPosition().in(Radians), TOLERANCE);
        assertEquals(0.0, result.wristMotorPosition().in(Radians), TOLERANCE);
    }
}
