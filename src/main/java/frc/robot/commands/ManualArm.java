package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ManualArm extends Command {
  private final Arm mArm;
  //private final boolean mIsUp;
  private final double mSpeed;

  public ManualArm(Arm arm, double speed) {
    mArm = arm;
    mSpeed = speed;

    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mWrist.setPistonsState(mIsUp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mArm.MoveArm(mSpeed);
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
