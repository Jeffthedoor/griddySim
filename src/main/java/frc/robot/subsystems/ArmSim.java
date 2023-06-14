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


public class ArmSim extends SubsystemBase {
    // The P gain for the PID controller that drives this arm.
    private double armKp = SimulationConstants.UP_kP;
    private double targetAngle = -113;

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor armGearbox = DCMotor.getNEO(2);
    private final ArmFeedforward armFF = new ArmFeedforward(0.0, 0.02, 0.0);

    // Standard classes for controlling our arm
    private final PIDController controller = new PIDController(0.08, 0, 0);
    private final Encoder encoder = new Encoder(2, 3);
    private final PWMSparkMax motor = new PWMSparkMax(1);
    
    private final SingleJointedArmSim armSim =
        new SingleJointedArmSim(
            armGearbox,
            200,
            SingleJointedArmSim.estimateMOI(0.67, 10),
            0.67,
            Math.toRadians(-127),
            Math.toRadians(180),
            true,
            VecBuilder.fill(Math.PI / 4096) // Add noise with a std-dev of 1 tick
            );
    private final EncoderSim encoderSim = new EncoderSim(encoder);

    

    /** Subsystem constructor. */
    public ArmSim() {
        encoder.setDistancePerPulse(2.0 * Math.PI / 4096);
}

    /** Update the simulation model. */
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)

        if(DriverStation.isEnabled()) {
        armSim.setInput(motor.get() * RobotController.getBatteryVoltage());
        } else {
            armSim.setInput(0);
        }

        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderSim.setDistance(armSim.getAngleRads());
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }

    @Override
    public void periodic() {
        // controller.setP(LightningShuffleboard.getDouble("Arm", "kP", 0.0));

        // setTarget(LightningShuffleboard.getDouble("Arm", "target", 0d));

        doPid();

        LightningShuffleboard.setDouble("Arm", "current", Math.toDegrees(armSim.getAngleRads()));
    }

    public void setTarget(double target) {
        targetAngle = target;

        

        //  LightningShuffleboard.setDouble("Arm", "PID out", pidOutput);
        //  LightningShuffleboard.setDouble("Arm", "F out", FFOut);

    }

    public void doPid() {
        double pidOutput = controller.calculate(Math.toDegrees(armSim.getAngleRads()), targetAngle);

        // double FFOut = new ArmFeedforward(LightningShuffleboard.getDouble("Arm", "kS", 0.0), LightningShuffleboard.getDouble("Arm", "kG", 0.0), LightningShuffleboard.getDouble("Arm", "kV", 0.0)).calculate(target, Math.toDegrees(armSim.getVelocityRadPerSec()));
        double FFOut = armFF.calculate(Math.toDegrees(armSim.getAngleRads()), Math.toDegrees(armSim.getVelocityRadPerSec()));
        
        motor.set(pidOutput + FFOut);
    }

    public boolean onTarget() {
        return Math.abs(getAngle() - targetAngle) < ArmConstants.TOLERANCE;
    }

    public boolean onTarget(double target) {
        return Math.abs(getAngle() - target) < ArmConstants.TOLERANCE;
    }

    public void setAngle(Rotation2d angle) {
        setTarget(angle.getDegrees());
    }

    public double getAngle() {
        return Math.toDegrees(armSim.getAngleRads());
    }

    public void stop() {
        motor.set(0.0);
    }
}