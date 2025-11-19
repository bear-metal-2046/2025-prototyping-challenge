package org.tahomarobotics.robot.endeffector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import static edu.wpi.first.units.Units.Volt;

public class EndEffectorSubsystem extends AbstractSubsystem {

    private final TalonFX endEffectorMotor;
    
    // State tracking for logging
    private double lastLoggedVoltage = 0.0;
    private double lastPeriodicLogTime = 0.0;
    private double lastVoltageDeltaLogTime = 0.0;

    public EndEffectorSubsystem() {
        endEffectorMotor = new TalonFX(RobotMap.ENDEFFECTOR_MOTOR);
        RobustConfigurator.tryConfigureTalonFX("End Effector Motor", endEffectorMotor, EndEffectorConstants.endEffectorMotorConfig);
        
        // Structured lifecycle logging
        logLifecycleEvent("init", "EndEffectorSubsystem initialized");
        org.tinylog.Logger.info("EndEffectorSubsystem initialized - Motor ID: {}, Sim: {}", 
            RobotMap.ENDEFFECTOR_MOTOR, RobotBase.isSimulation());
    }

    public void setVoltage(Voltage voltage) {
        double voltageValue = voltage.in(Volt);
        double previousVoltage = lastLoggedVoltage;
        endEffectorMotor.setVoltage(voltageValue);
        
        // Log significant voltage changes
        double voltageDelta = Math.abs(voltageValue - previousVoltage);
        if (voltageDelta >= EndEffectorConstants.VOLTAGE_DELTA_THRESHOLD) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - lastVoltageDeltaLogTime >= EndEffectorConstants.RATE_LIMIT_INTERVAL) {
                org.tinylog.Logger.debug("EndEffector voltage change: {:.2f}V -> {:.2f}V (delta: {:.2f}V)", 
                    previousVoltage, voltageValue, voltageDelta);
                logActuatorDelta(previousVoltage, voltageValue);
                lastVoltageDeltaLogTime = currentTime;
            }
        }
        
        lastLoggedVoltage = voltageValue;
    }

    @Override
    public void subsystemPeriodic() {
        // Get motor status signals
        StatusSignal<Double> motorVoltage = endEffectorMotor.getMotorVoltage();
        StatusSignal<Double> statorCurrent = endEffectorMotor.getStatorCurrent();
        StatusSignal<Double> supplyCurrent = endEffectorMotor.getSupplyCurrent();
        StatusSignal<Double> deviceTemp = endEffectorMotor.getDeviceTemp();
        StatusSignal<Double> position = endEffectorMotor.getPosition();
        StatusSignal<Double> velocity = endEffectorMotor.getVelocity();
        
        // Log to AdvantageKit for replay capability
        Logger.recordOutput("EndEffector/Voltages/Motor", motorVoltage.getValue());
        Logger.recordOutput("EndEffector/Currents/Stator", statorCurrent.getValue());
        Logger.recordOutput("EndEffector/Currents/Supply", supplyCurrent.getValue());
        Logger.recordOutput("EndEffector/Temperature/Motor", deviceTemp.getValue());
        Logger.recordOutput("EndEffector/Position", position.getValue());
        Logger.recordOutput("EndEffector/Velocity", velocity.getValue());
        
        // Periodic snapshot logging (rate-limited)
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastPeriodicLogTime >= EndEffectorConstants.PERIODIC_LOG_INTERVAL) {
            org.tinylog.Logger.trace("EndEffector periodic: voltage={:.2f}V, current={:.2f}A, temp={:.1f}C", 
                motorVoltage.getValue(), statorCurrent.getValue(), deviceTemp.getValue());
            lastPeriodicLogTime = currentTime;
        }
    }
    
    /**
     * Logs lifecycle events to AdvantageKit with structured data
     */
    private void logLifecycleEvent(String event, String message) {
        Logger.recordOutput("EndEffector/Lifecycle/Event", event);
        Logger.recordOutput("EndEffector/Lifecycle/Message", message);
        Logger.recordOutput("EndEffector/Lifecycle/Timestamp", Timer.getFPGATimestamp());
        Logger.recordOutput("EndEffector/Lifecycle/IsSimulation", RobotBase.isSimulation());
    }
    
    /**
     * Logs actuator output delta events to AdvantageKit
     */
    private void logActuatorDelta(double previousVoltage, double newVoltage) {
        Logger.recordOutput("EndEffector/ActuatorDelta/PreviousVoltage", previousVoltage);
        Logger.recordOutput("EndEffector/ActuatorDelta/NewVoltage", newVoltage);
        Logger.recordOutput("EndEffector/ActuatorDelta/Delta", Math.abs(newVoltage - previousVoltage));
        Logger.recordOutput("EndEffector/ActuatorDelta/Timestamp", Timer.getFPGATimestamp());
    }
    
    /**
     * Called when subsystem is being shutdown
     */
    public void close() {
        logLifecycleEvent("shutdown", "EndEffectorSubsystem shutting down");
        org.tinylog.Logger.info("EndEffectorSubsystem shutdown");
    }
}
