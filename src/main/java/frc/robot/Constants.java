// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SimulationConstants {
     // PID gains for our wrist going up
     public static final double UP_kP = 0.0072d;
     public static final double UP_kD = 0.0001d;
     public static final double DOWN_kP = 0.005d;
     public static final double DOWN_kD = 0d;
     public static final double kI = 0d;

     // Min/max angles in degrees
     public static final double MAX_ANGLE = 126d;
     public static final double MIN_ANGLE = -90d;


     // Minimum length of elevator in inches, might need double checking
     public static final double MIN_ELEVATOR_LENGTH = 36.5d;
     public static final double ARM_LENGTH = 26.519; 
     public static final double WRIST_LENGTH = 9.75d;
  }

  // Constants for the lift
    public static final class LiftConstants {
        public double ELEVATOR_SAFE_HEIGHT = 4;

        // All of the different states the lift can be in
        public enum LiftState {
            //ground collects
            groundCone, groundCube,

            //substation collects (TODO: see if we need seperate setpoints/states for cube vs cone)
            doubleSubstationCollect, singleSubCone, singleSubCube, OTB_DoubleSubstationCollect,

            //score states
            midCubeScore, highCubeScore, midConeScore, highConeScore, OTB_Mid, OTB_High,

            //substates
            stowedCollect, stowedScore, stowedSingleSub, scoreToCollect, elevatorDeployed,

            //s t o w e d
            stowed
        }

        // All of the different plans the lift can follow
        public enum LiftPlan {
            parallel, armThenWristAndEle, eleWristArm, eleArmWrist, armAndWristThenEle, eleThenArmAndWrist, eleAndWristThenArm, wristArmEle
        }
        public static final double LOG_PERIOD = 0.23;
    }


    // Constants for our elevator
    public static final class ElevatorConstants {
        // Motor configuration constants
        public static final boolean MOTOR_INVERT = false;
        public static final int CURRENT_LIMIT = 40;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        // PID gains for our elevator
        public static final double kP = .35d;
        public static final double kI = 0d;
        public static final double kD = 0d;
        public static final double kF = 0.007d;

        // TODO: set a tolerance
        public static final double TOLERANCE = 1d;

        // Conversion factor for our elevator
        public static final double GEAR_RATIO = 16d / 1d; // Motor gear reduction / output shaft gear reduction
        public static final double SPROCKET_DIAMETER = 1.440d;
        public static final double POSITION_CONVERSION_FACTOR = 1 / GEAR_RATIO * SPROCKET_DIAMETER * Math.PI;

        // Min/max height in inches
        public static final double MAX_EXTENSION = 24;
        public static final double MIN_EXTENSION = 0d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 1d;

        public static final double LOG_PERIOD = 0.19;

        // Elevator limit switch types
        public static final SparkMaxLimitSwitch.Type TOP_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type BOTTOM_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
    }

    public static final class LimelightConstants {
        public static final String FRONT_NAME = "limelight-front";
        public static final String BACK_NAME = "limelight-back";
        public static final Pose3d FRONT_POSE = new Pose3d(.1, 0.28, 0.72, new Rotation3d(0, 0, 0));
        public static final Pose3d BACK_POSE = new Pose3d(.1, 0.28, 0.83, new Rotation3d(0, 10, 180));
        public static final double CUBE_OFFSET = 0.0; // TODO find this value
    }

    public static final class ArmConstants {
        // Motor configuration constants
        public static final boolean MOTOR_INVERT = true;
        public static final int CURRENT_LIMIT = 50;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        // PID gains for our arm
        public static final double UP_kP = 0.013d;
        public static final double UP_kI = 0d;
        public static final double UP_kD = 0d;

        public static final double DOWN_kP = 0.01d;
        public static final double DOWN_kI = 0d;
        public static final double DOWN_kD = 0d;
        public static final double kF = 0d;

        public static final double TOLERANCE = 10d;

        // Min and Max arm angles in degrees
        // TODO: change to actual values
        public static final double MAX_ANGLE = 180;
        public static final double MIN_ANGLE = -113d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 1d;

        public static final double LENGTH = 26.519; // arm length in inches

        // Offsets in degrees
        public static final double ENCODER_OFFSET_GRIDLOCK = -104.68;
        public static final double ENCODER_OFFSET_BLACKOUT = 89.8;

        // Conversion factor for our arm, multiply this by the navite units to get degrees
        public static final double POSITION_CONVERSION_FACTOR = 360;

        // Arm limit switch types
        public static final SparkMaxLimitSwitch.Type TOP_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type BOTTOM_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;

        public static final double LOG_PERIOD = 0.21;
    }

    public static final class CollectorConstants {
        public static final boolean MOTOR_INVERT = false;
        public static final int CURRENT_LIMIT = 30;
        public static final double HOLD_POWER_CUBE = 0.25;
        public static final double HOLD_POWER_CONE = 0.35;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        public static final double STALL_POWER = 35d;

        public static final double LOG_PERIOD = 0.22;

        // // Enum of possible game pieces
        // public enum GamePiece {
        //     CONE, CUBE, NONE
        // }

        //TODO: tune these
        //Cube Theoretical: #3a01b2 (58, 1, 178)
        public static final Color CUBE_OPTIMAL = new Color(58, 1, 178);
        //Cone Theretical: #cb6200 (203,98,0)
        public static final Color CONE_OPTIMAL = new Color(203, 98, 0);
    }

    public static final class WristConstants {

        // Motor configuration constants
        public static final boolean MOTOR_INVERT = true;
        public static final int CURRENT_LIMIT = 30;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;

        // PID gains for our wrist going up
        public static final double UP_kP = 0.0072d;
        public static final double UP_kD = 0.0001d;
        public static final double DOWN_kP = 0.005d;
        public static final double DOWN_kD = 0d;
        public static final double kI = 0d;

        // Tolernace for our wrist
        public static final double TOLERANCE = 12d;

        // Min/max angles in degrees
        public static final double MAX_ANGLE = 126d;
        public static final double MIN_ANGLE = -90d;

        // Min and Max power
        public static final double MIN_POWER = -1d;
        public static final double MAX_POWER = 0.9d;

        public static final double LOG_PERIOD = 0.24;

        // Offsets in degrees        
        public static final double ENCODER_OFFSET_GRIDLOCK = -131d;

        public static final double ENCODER_OFFSET_BLACKOUT = -22; //TODO: change

        // Conversion factor for our wrist, multiply this by the navite units to get degrees
        public static final double POSITION_CONVERSION_FACTOR = 360;

        // Wrist limit switch types
        public static final SparkMaxLimitSwitch.Type TOP_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
        public static final SparkMaxLimitSwitch.Type BOTTOM_LIMIT_SWITCH_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
    }
}
