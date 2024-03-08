package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooterRPM extends Command {
  private final Shooter mShooter;
  //private final boolean mIsUp;
  private final double mTopSpeed;
  //private final double mBottomSpeed;

  public RunShooterRPM(Shooter shooter, double topSpeed) {
    mShooter = shooter;
    mTopSpeed = topSpeed;

    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mWrist.setPistonsState(mIsUp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTopSpeed != 0){
      mShooter.RunShooterRPM(mTopSpeed);
    }
    else{
      mShooter.StopShooter();
    }
    //SmartDashboard.putData(mShooter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
