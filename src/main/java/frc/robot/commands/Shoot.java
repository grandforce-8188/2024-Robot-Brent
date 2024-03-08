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



import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.MathEquations;

public class Shoot extends Command {
  //private final Shooter mShooter;
  //private final boolean mIsUp;
  private final Shooter mShooter;
  private final Intake mIntake;
  private final Feeder mFeeder;
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


  public Shoot(Shooter shooter, Intake intake, Feeder feeder) {
    //mShooter = shooter;
    mShooter = shooter;
    mIntake = intake;
    mFeeder = feeder;
    // mDrivetrain = drivetrain;
    // mJoystick = joystick;
    // mDrive = drive;
    
    addRequirements(mIntake, mFeeder);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mWrist.setPistonsState(mIsUp);
    // mDrivetrain.setDefaultCommand(mDrivetrain.applyRequest(() -> mDrive.withVelocityX(Math.pow(-mJoystick.getLeftY(), 3) * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(Math.pow(-mJoystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(rotationValue * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));
    // mDrivetrain.setDefaultCommand(mDrivetrain.applyRequest(mDrive.apply(Math.pow(-mJoystick.getLeftY(), 3) * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(Math.pow(-mJoystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(MathUtil.applyDeadband(-mJoystick.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));
    //mDrive.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        mShooter.RunShooterRPM(80);
        mIntake.RunIntake(-1.0);
        mFeeder.RunFeeder(-1.0);
      }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // mDrivetrain.setDefaultCommand(mDrivetrain.applyRequest(() -> mDrive.withVelocityX(Math.pow(-mJoystick.getLeftY(), 3) * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(Math.pow(-mJoystick.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(MathUtil.applyDeadband(-mJoystick.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));
    mShooter.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
