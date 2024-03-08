package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ManualArm extends Command {
  private final Arm mArm;
  //private final Shooter mShooter;
  //private final boolean mIsUp;
  private final CommandJoystick mJoystick;

  public ManualArm(Arm arm, CommandJoystick joystick) {
    mArm = arm;
    mJoystick = joystick;
    //mShooter = shooter;

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
    // if (mSpeed > 0 && !mShooter.getBottomArmSensor())
    //   mArm.ManualMoveArm(mSpeed);
    // if (mSpeed < 0 && !mShooter.getTopArmSensor()){
    //   mArm.ManualMoveArm(mSpeed);
    // }
    // if (mSpeed == 0){
    //   mArm.ManualMoveArm(mSpeed);
    // }
    double speed = mJoystick.getY();

    mArm.ManualMoveArm(speed);

    //SmartDashboard.putData("Arm position", mArm.GetEncoderPosition());
    SmartDashboard.putNumber("Arm position", mArm.GetEncoderPosition());
    SmartDashboard.putBoolean("Top Limit Switch", mArm.GetTopLimitSwitch());
    SmartDashboard.putBoolean("Bottom Limit Switch", mArm.GetBottomLimitSwitch());
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
