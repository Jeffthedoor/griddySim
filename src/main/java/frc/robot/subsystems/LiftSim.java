// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.Lift.StateTable;
import frc.robot.commands.Lift.StateTransition;

public class LiftSim extends SubsystemBase {


    // The current and goal states
    private LiftState currentState = LiftState.stowed;
    private LiftState goalState = LiftState.stowed;

    private boolean doTargetOverride = false;

    private double lastKnownGoodWristSetPoint = 0;

    // The next state to transition to
    private StateTransition nextState;
  
  // Create a Mechanism2d display of an wrist with a fixed wristTower and moving wrist.

    private final Mechanism2d mech2d = new Mechanism2d(90, 90);
    private final MechanismRoot2d eleRoot = mech2d.getRoot("Elevator Root", 13, 5);
  private final MechanismLigament2d elevatorMech2d =
      eleRoot.append(
          new MechanismLigament2d("Elevator", 37, 55));
    private final MechanismLigament2d stageOne = eleRoot.append(new MechanismLigament2d("Stage One", 37, 55, 12, new Color8Bit(Color.kRed)));
    // private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("Elevator", 30, 235));
    private final MechanismLigament2d armMech =
        elevatorMech2d.append(
            new MechanismLigament2d(
                "Arm",
                30,
                Units.radiansToDegrees(-55),
                6,
                new Color8Bit(Color.kYellow)));
    private final MechanismLigament2d wristMech =
        armMech.append(
            new MechanismLigament2d(
                "Wrist",
                10,
                Units.radiansToDegrees(0),
                6,
                new Color8Bit(Color.kAquamarine)));

    private final MechanismRoot2d highNodeHome = mech2d.getRoot("High Node", highNodeHomeX, 0);
    private final MechanismRoot2d midNodeHome = mech2d.getRoot("Mid Node", midNodeHomeX, 0);
    private final MechanismRoot2d gridHome = mech2d.getRoot("Grid Home", gridHomeX, 0);
    // private final MechanismRoot2d bumperRoot = eleRoot.append(new MechanismRoot2d("Grid Home", 40.25, 0));
    private final MechanismLigament2d MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismLigament2d HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismLigament2d GridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 0, 50, new Color8Bit(Color.kWhite)));
    private final MechanismLigament2d bumper = eleRoot.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));

    private final ArmSim arm;
    private final WristSim wrist;
    private final Elevator elevator;

    private DoubleSupplier deltaX;
    private BooleanSupplier isJumping;
    private double mechX = 13; 
    private double mechY = 5;
    private boolean allowJump = true;

    private boolean scene = false;

    public LiftSim(ArmSim arm, WristSim wrist, Elevator elevator, DoubleSupplier deltaX, BooleanSupplier isJumping) {
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", mech2d);

        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;
        this.deltaX = deltaX;
        this.isJumping = isJumping;
    }

    @Override
    public void simulationPeriodic() {
        wristMech.setAngle(wrist.getAngle());
        armMech.setAngle(arm.getAngle() - 55);
        elevatorMech2d.setLength(elevator.getExtension() + 37);

        if(isJumping.getAsBoolean() && mechY <= 30 && allowJump) {
            mechY += 4;            
        } else if(mechY > 5) {
            mechY -= 0.5;
            allowJump = false;
        } else if (mechY == 5) {
            allowJump = true;
        } 

        if(mechX + 30.5 == 90 || mechX == 0) {
            scene = !scene;
        } 
        gridHome.
        
        if(!(mechX + 30.5 >= gridHomeX && deltaX.getAsDouble() > 0) || (mechY >= 10)) {
            mechX += deltaX.getAsDouble();
        }
        eleRoot.setPosition(mechX, mechY);
    }



    /**
     * Sets the goal state of the lift
     * 
     * @param state the goal state
     */
    public void setGoalState(LiftState state) {
        this.goalState = state;
    }

    public LiftState getCurrentState() {
        return currentState;
    }

    public LiftState getGoalState() {
        return goalState;
    }

    /**
     * Checks if the all the components of lift are on target
     * 
     * @return true if all the components of lift are on target
     */
    public boolean onTarget() {
        if (nextState == null) {
            return elevator.onTarget() && arm.onTarget() && wrist.onTarget();
        } else if (doTargetOverride) {
            doTargetOverride = false;
            return true;
        } else {
            return elevator.onTarget(nextState.getElevatorExtension()) && arm.onTarget(nextState.getArmAngle().getDegrees()) && wrist.onTarget(nextState.getWristAngle().getDegrees());
        }
    }

    public boolean goalReached() {
        return currentState == goalState;
    }

    public void breakLift() {
        elevator.setTarget(elevator.getExtension());
        arm.setTarget(arm.getAngle());
        wrist.setTarget(wrist.getAngle());

        nextState = null;
    }

    public void stop() {
        elevator.stop();
        arm.stop();
        wrist.stop();
    }

    @Override
    public void periodic() {

        // Checks if were on target or if the next state is null
        // Checks if were on target or if the next state is null, also makes sure our biassese havent changed
        if (onTarget() || nextState == null) {
            // Checks if the current state is not the goal state
            if (currentState != goalState) {
                // Gets the next state from the state table
                nextState = StateTable.get(currentState, goalState);

            } else {
                // sets the next state to null
                nextState = null;
            }

            // Checks if the next state is not null
            if (nextState != null) {
                // Sets the current state to the end state of the next state
                currentState = nextState.getEndState();
            }
        } else {
            lastKnownGoodWristSetPoint = nextState.getWristAngle().getDegrees();
            // Checks the run plan of the next state
            switch (nextState.getPlan()) {
                // If parallel, set all the components to their target
                case parallel:
                    elevator.setTarget(nextState.getElevatorExtension());
                    arm.setAngle(nextState.getArmAngle());
                    wrist.setAngle(nextState.getWristAngle());
                    break;
                case armThenWristAndEle:
                    arm.setAngle(nextState.getArmAngle());
                    if (nextState.isInArmSafeZone(arm.getAngle())) {
                        elevator.setTarget(nextState.getElevatorExtension());
                        wrist.setAngle(nextState.getWristAngle());
                    }
                    break;
                case eleWristArm:
                    elevator.setTarget(nextState.getElevatorExtension());
                    if (nextState.isInEleSafeZone(elevator.getExtension())) {
                        wrist.setAngle(nextState.getWristAngle());
                        if (nextState.isInWristSafeZone(wrist.getAngle())) {
                            arm.setAngle(nextState.getArmAngle());
                        }
                    }
                    break;
                case eleArmWrist:
                    elevator.setTarget(nextState.getElevatorExtension());
                    if (nextState.isInEleSafeZone(elevator.getExtension())) {
                        arm.setAngle(nextState.getArmAngle());
                        if (nextState.isInArmSafeZone(arm.getAngle())) {
                            wrist.setAngle(nextState.getWristAngle());
                        }
                    }
                    break;
                case armAndWristThenEle:
                    arm.setAngle(nextState.getArmAngle());
                    wrist.setAngle(nextState.getWristAngle());
                    if (nextState.isInArmSafeZone(arm.getAngle()) && nextState.isInWristSafeZone(wrist.getAngle())) {
                        elevator.setTarget(nextState.getElevatorExtension());
                    }
                    break;
                case eleThenArmAndWrist:
                    elevator.setTarget(nextState.getElevatorExtension());
                    if (nextState.isInEleSafeZone(elevator.getExtension())) {
                        arm.setAngle(nextState.getArmAngle());
                        wrist.setAngle(nextState.getWristAngle());
                    }
                    break;
                case eleAndWristThenArm:
                    elevator.setTarget(nextState.getElevatorExtension());
                    wrist.setAngle(nextState.getWristAngle());
                    if (nextState.isInEleSafeZone(elevator.getExtension()) && nextState.isInWristSafeZone(wrist.getAngle())) {
                        arm.setAngle(nextState.getArmAngle());
                    }
                    break;
                case wristArmEle:
                    wrist.setAngle(nextState.getWristAngle());
                    if (nextState.isInWristSafeZone(wrist.getAngle())) {
                        arm.setAngle(nextState.getArmAngle());
                        if (nextState.isInArmSafeZone(arm.getAngle())) {
                            elevator.setTarget(nextState.getElevatorExtension());
                        }
                    }
                    break;
            }
        }

        if (nextState != null) {
            LightningShuffleboard.setBool("Lift", "ele on targ", elevator.onTarget(nextState.getElevatorExtension()));
            LightningShuffleboard.setBool("Lift", "arm on targ", arm.onTarget(nextState.getArmAngle().getDegrees()));
            LightningShuffleboard.setBool("Lift", "wrist on targ", wrist.onTarget(nextState.getWristAngle().getDegrees()));
            LightningShuffleboard.setDouble("Lift", "arm target", nextState.getArmAngle().getDegrees());
            LightningShuffleboard.setDouble("Lift", "ele target", nextState.getElevatorExtension());
            LightningShuffleboard.setDouble("Lift", "wrist target", nextState.getWristAngle().getDegrees());

            LightningShuffleboard.setBool("Lift", "ele in safe zone", nextState.isInEleSafeZone(elevator.getExtension()));
            LightningShuffleboard.setBool("Lift", "arm in safe zone", nextState.isInArmSafeZone(arm.getAngle()));
            LightningShuffleboard.setBool("Lift", "wrist in safe zone", nextState.isInWristSafeZone(wrist.getAngle()));

        }
    }
}
