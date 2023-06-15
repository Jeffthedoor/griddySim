package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.LiftSim;

/**
 * Sets the lift position to the high score position for cubes or cones depending on readings from
 * the color sensor
 */
public class MidScore extends InstantCommand {
    private LiftSim lift;
    private DoubleSupplier gamePiece;

    public MidScore(LiftSim lift, DoubleSupplier gamePiece) {
        this.lift = lift;
        this.gamePiece = gamePiece;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (gamePiece.getAsDouble() == 0) {
            lift.setGoalState(LiftState.midConeScore);
        } else {
            lift.setGoalState(LiftState.midCubeScore);
        }
    }
}
