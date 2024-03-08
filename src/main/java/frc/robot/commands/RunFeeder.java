package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends Command {
  private final Feeder mFeeder;
  //private final boolean mIsUp;
  private final double mSpeed;

  public RunFeeder(Feeder feeder, double speed) {
    mFeeder = feeder;
    mSpeed = speed;

    addRequirements(mFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mWrist.setPistonsState(mIsUp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mFeeder.RunFeeder(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
