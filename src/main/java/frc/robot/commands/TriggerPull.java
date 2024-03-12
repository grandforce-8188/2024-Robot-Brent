package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.math.MathUtil;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.MathEquations;
import frc.robot.commands.RunShooterRPM;

public class TriggerPull extends Command {
  private final Arm mArm;
  //private final Shooter mShooter;
  //private final boolean mIsUp;
  private final Shooter mShooter;
  private final Intake mIntake;
  private final Feeder mFeeder;
  private final Limelight mLimelight;
  private final Swerve mDrivetrain;
  // private CommandSwerveDrivetrain mDrivetrain;
  // private final CommandXboxController mJoystick;
  private double rotationValue = 0;

  private final MathEquations mathEquations = new MathEquations();


  // private double MaxSpeed = 6;
  // private double MaxAngularRate = 1.5 * Math.PI;
  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.FieldCentric mDrive;


  public TriggerPull(Arm arm, Shooter shooter, Intake intake, Feeder feeder, Limelight limelight, Swerve driSwerve) {
    mArm = arm;
    //mShooter = shooter;
    mShooter = shooter;
    mIntake = intake;
    mFeeder = feeder;
    mLimelight = limelight;
    mDrivetrain = driSwerve;
    // mDrivetrain = drivetrain;
    // mJoystick = joystick;
    // mDrive = drive;
    
    

    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (mArm.GetEncoderPosition() > 80){
        mArm.PIDMoveArm(Constants.upperArmPosLimit);
        mIntake.RunIntake(-1.0);
        mFeeder.RunFeeder(-0.5);
        mShooter.RunShooterRPM(10);
      }
      else{
        double speakerOffset = mLimelight.getSpeakerCenter();
        rotationValue = speakerOffset * -0.15;

        mDrivetrain.drive(new Translation2d(0, 0), rotationValue, false, false);

        double speakerDistance = mLimelight.getSpeakerDistance();
        double armAngle = mathEquations.getShootingArmAngle(speakerDistance);
        double shooterSpeed = mathEquations.getShootingSpeed(speakerDistance);
        // SmartDashboard.putNumber("Shooting angle", armAngle);
        SmartDashboard.putNumber("Speaker distance", speakerDistance);
        // SmartDashboard.putNumber("Speaker offset", speakerOffset);
        // SmartDashboard.putNumber("Commanded turn", rotationValue);
        // SmartDashboard.putNumber("Commanded speed", shooterSpeed);

        mArm.PIDMoveArm(armAngle);
        //mShooter.run(runShooter(shooterSpeed));
        mShooter.RunShooterRPM(90);
        double armError = Math.abs(mArm.GetEncoderPosition() - armAngle);
        double shooterRPM = mShooter.getTopRPM();

        if (armError < 2 && speakerOffset < 1.5 && shooterRPM > 80){

          mIntake.RunIntake(-Constants.intakeSpeed);
          mFeeder.RunFeeder(-1.0);
        }        
      }

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
