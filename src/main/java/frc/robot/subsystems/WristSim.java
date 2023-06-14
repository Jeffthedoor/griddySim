// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.WristConstants;


public class WristSim extends SubsystemBase {

    double targetAngle = 112;

    // The P gain for the PID controller that drives this wrist.
    ArmSim arm;
    // The wrist gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor wristGearbox = DCMotor.getNeo550(1);
    private final ArmFeedforward wristFF = new ArmFeedforward(0.0, 0.02, 0.0);

    // Standard classes for controlling our wrist
    private final PIDController controller = new PIDController(0.08, 0, 0);
    private final Encoder encoder = new Encoder(4, 5);
    private final PWMSparkMax motor = new PWMSparkMax(6);
    
    private final SingleJointedArmSim wristSim =
        new SingleJointedArmSim(
            wristGearbox,
            100,
            SingleJointedArmSim.estimateMOI(0.26, 4),
            0.26,
            Math.toRadians(-90),
            Math.toRadians(126),
            true,
            VecBuilder.fill(Math.PI / 4096) // Add noise with a std-dev of 1 tick
            );
    private final EncoderSim encoderSim = new EncoderSim(encoder);

    
    /** Subsystem constructor. */
    public WristSim(ArmSim arm) {
        encoder.setDistancePerPulse(2.0 * Math.PI / 4096);
        this.arm = arm;
}

    /** Update the simulation model. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our wrist is doing
        // First, we set our "inputs" (voltages)
        if(DriverStation.isEnabled()) {
        wristSim.setInput(motor.get() * RobotController.getBatteryVoltage());
        } else {
            wristSim.setInput(0);
        }

        // Next, we update it. The standard loop time is 20ms.
        wristSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setDistance(wristSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }

    @Override
    public void periodic() {
        // controller.setP(LightningShuffleboard.getDouble("wrist", "kP", 0.0));

        // setTarget(LightningShuffleboard.getDouble("wrist", "target", 0d));

        LightningShuffleboard.setDouble("wrist", "current", Math.toDegrees(wristSim.getAngleRads()));

        doPid();
    }

    public void setTarget(double target) {
        targetAngle = target;

        

        //  LightningShuffleboard.setDouble("wrist", "PID out", pidOutput);
        //  LightningShuffleboard.setDouble("wrist", "F out", FFOut);

    }

    public double getAngle() {
        return Math.toDegrees(wristSim.getAngleRads());
    }

    public void setAngle(Rotation2d angle) {
        setTarget(angle.getDegrees());
    }

    private void doPid() {
        double pidOutput = controller.calculate(Math.toDegrees(wristSim.getAngleRads()), targetAngle);

        // double FFOut = new ArmFeedforward(LightningShuffleboard.getDouble("wrist", "kS", 0.0), LightningShuffleboard.getDouble("wrist", "kG", 0.0), LightningShuffleboard.getDouble("wrist", "kV", 0.0)).calculate(target, Math.toDegrees(wristSim.getVelocityRadPerSec()));
        double FFOut = wristFF.calculate(Math.toDegrees(wristSim.getAngleRads()) + arm.getAngle(), Math.toDegrees(wristSim.getVelocityRadPerSec()));
        
        motor.set(pidOutput + FFOut);
    }

    
    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < WristConstants.TOLERANCE;
    }

    public boolean onTarget(double target) {
        return Math.abs(getAngle() - target) < WristConstants.TOLERANCE;
    }

    public void stop() {
        motor.set(0.0);
    }
}