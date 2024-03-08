// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;

// public class RunShooterPercent extends Command {
//   private final Shooter mShooter;
//   //private final boolean mIsUp;
//   private final double mTopSpeed;
//   private final double mBottomSpeed;

//   public RunShooterPercent(Shooter shooter, double topSpeed, double bottomSpeed) {
//     mShooter = shooter;
//     mTopSpeed = topSpeed;
//     mBottomSpeed = bottomSpeed;

//     addRequirements(mShooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     //mWrist.setPistonsState(mIsUp);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     mShooter.RunShooterPercent(mTopSpeed, mBottomSpeed);
//     System.out.println(mBottomSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
