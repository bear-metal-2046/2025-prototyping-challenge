package org.tahomarobotics.robot.arm;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;


public class ArmDifferentialTransform {

    public static final DCMotor KRACKEN_X60_FOC = DCMotor.getKrakenX60Foc(2);

    public record DiffMotorVoltages(Voltage topMotorVoltage, Voltage bottomMotorVoltage) {}
    public record DiffMotorPositions(Angle topMotorPosition, Angle bottomMotorPosition) {}
    public record DiffMotorVelocities(AngularVelocity topMotorAngularVelocity, AngularVelocity bottomMotorAngularVelocity) {}
    public record VirtualMotorVoltages(Voltage elbowMotorVoltage, Voltage wristMotorVoltage) {}
    public record VirtualMotorPositions(Angle elbowMotorPosition, Angle wristMotorPosition) {}
    public record VirtualMotorVelocities(AngularVelocity elbowAngularVelocity, AngularVelocity wristAngularVelocity) {}

    public static VirtualMotorVoltages transform(DiffMotorVoltages diffMotorVoltages, DiffMotorVelocities diffMotorVelocities) {    

        // Convert motor voltages to currents
        double topCurrent = KRACKEN_X60_FOC.getCurrent(
            diffMotorVelocities.topMotorAngularVelocity.in(RadiansPerSecond), 
            diffMotorVoltages.topMotorVoltage.in(Volts));
        
        double bottomCurrent = KRACKEN_X60_FOC.getCurrent(
            diffMotorVelocities.bottomMotorAngularVelocity.in(RadiansPerSecond), 
            diffMotorVoltages.bottomMotorVoltage.in(Volts));
        

        // Differential kinematics
        double elbowTorque = KRACKEN_X60_FOC.getTorque(topCurrent + bottomCurrent);
        double wristTorque = KRACKEN_X60_FOC.getTorque(topCurrent - bottomCurrent);

        // need virtual motor velocities
        VirtualMotorVelocities virtualMotorVelocities = transform(diffMotorVelocities);

        // Convert torques back to voltages
        double elbowVoltage = KRACKEN_X60_FOC.getVoltage(elbowTorque, virtualMotorVelocities.elbowAngularVelocity().in(RadiansPerSecond));
        double wristVoltage = KRACKEN_X60_FOC.getVoltage(wristTorque, virtualMotorVelocities.wristAngularVelocity().in(RadiansPerSecond));
        
        return new VirtualMotorVoltages(Volts.of(elbowVoltage), Volts.of(wristVoltage));
    }

    public static VirtualMotorVelocities transform(DiffMotorVelocities diffMotorVelocities) {

        double top = diffMotorVelocities.topMotorAngularVelocity.in(RadiansPerSecond);
        double bottom = diffMotorVelocities.bottomMotorAngularVelocity.in(RadiansPerSecond);

        AngularVelocity elbow = RadiansPerSecond.of((top + bottom)/2d);
        AngularVelocity wrist = RadiansPerSecond.of((top - bottom)/2d);
        
        return new VirtualMotorVelocities(elbow, wrist);
    }

    public static DiffMotorVelocities transform(VirtualMotorVelocities virtualMotorVelocities) {

        double elbow = virtualMotorVelocities.elbowAngularVelocity.in(RadiansPerSecond);
        double wrist = virtualMotorVelocities.wristAngularVelocity.in(RadiansPerSecond);

        AngularVelocity top = RadiansPerSecond.of(elbow + wrist);
        AngularVelocity bottom = RadiansPerSecond.of(elbow - wrist);

        return new DiffMotorVelocities(top, bottom);
    }

    public static DiffMotorPositions transform(VirtualMotorPositions virtualMotorPositions) {
        
        double elbow = virtualMotorPositions.elbowMotorPosition.in(Radians);
        double wrist = virtualMotorPositions.wristMotorPosition.in(Radians);

        Angle top = Radians.of(elbow + wrist);
        Angle bottom = Radians.of(elbow - wrist);

        return new DiffMotorPositions(top, bottom);
    }

    public static VirtualMotorPositions transform(DiffMotorPositions diffMotorPositions) {
        
        double top = diffMotorPositions.topMotorPosition.in(Radians);
        double bottom = diffMotorPositions.bottomMotorPosition.in(Radians);

        Angle elbow = Radians.of((top + bottom)/2d);
        Angle wrist = Radians.of((top - bottom)/2d);

        return new VirtualMotorPositions(elbow, wrist);
    }
}
