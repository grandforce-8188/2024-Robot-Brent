package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class getSpeakerCenter extends Command {
  private final Limelight mlimelight;

  public getSpeakerCenter(Limelight limelight) {
    mlimelight = limelight;
    

    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mFliper.setPistonState(mIsUp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mlimelight.getSpeakerCenter();
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
