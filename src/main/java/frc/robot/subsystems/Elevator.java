// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor elevatorGearbox = DCMotor.getNEO(4);

  double targetExtension = 0;
  // Standard classes for controlling our elevator
  private final PIDController controller = new PIDController(8, 0, 0);
  ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);
  private final Encoder encoder = new Encoder(8 ,9);
  private final PWMSparkMax motor = new PWMSparkMax(10);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          16,
          14,
          0.0508, //TODO: figure out what we should actually do with this value
          0,
          0.61,
          true,
          VecBuilder.fill(0.0001));
  private final EncoderSim encoderSim = new EncoderSim(encoder);
  private final PWMSim motorSim = new PWMSim(motor);

  /** Subsystem constructor. */
  public Elevator() {
    encoder.setDistancePerPulse(2.0 * Math.PI / 4096);

  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    if(DriverStation.isEnabled()) {
      elevatorSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    } else {
      elevatorSim.setInput(0);
    }
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setDistance(elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
      
    // setTarget(LightningShuffleboard.getDouble("Elevator", "target", 0));
    LightningShuffleboard.setDouble("Elevator", "position", getExtension());

    // controller.setP(LightningShuffleboard.getDouble("Elevator", "kP", 0));

    doPid();
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void setTarget(double target) {
    target = Units.inchesToMeters(target);

    targetExtension = target;
  }

  private void doPid() {
    // With the setpoint value we run PID control like normal
    double pidOutput = controller.calculate(elevatorSim.getPositionMeters(), targetExtension);
    double feedforwardOutput = feedforward.calculate(elevatorSim.getVelocityMetersPerSecond());
    
    motor.set(pidOutput + feedforwardOutput);
  }

  public double getExtension() {
    return Units.metersToInches(elevatorSim.getPositionMeters());
  }

    public boolean onTarget() {
        return Math.abs(targetExtension - getExtension()) < ElevatorConstants.TOLERANCE;
        // return true;
    }

    public boolean onTarget(double target) {
        return Math.abs(target - getExtension()) < ElevatorConstants.TOLERANCE;
        // return true;
    }

  /** Stop the control loop and motor output. */
  public void stop() {
    motor.set(0.0);
  }

}
