package frc.robot.commands;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSim;

public class LiftCommand extends CommandBase {
  LiftSim liftSim;

  public LiftCommand(LiftSim liftSim) {
    this.liftSim = liftSim;
    addRequirements(liftSim);
  }
  

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
