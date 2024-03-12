package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MiniPID;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    double rotationCorrect = 0;
    double previousYaw;
    double currentYaw;
    double gyroChange = 0.0;
    double goalYaw = 0.0;
    double previousRotation = 0;
    double commandedRotationVal = 0;
    double rotationVal = 0;


    double kP = 0.01; 
    double kI = 0;
    double kD = 0;

    private final MiniPID mPID = new MiniPID(kP, kI, kD);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        currentYaw = s_Swerve.getGyroYaw().getDegrees();

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);


        
    }
                            

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        //double current_Yaw = Double.parseDouble(s_Swerve.gyro.getYaw().toString());

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        // if((p != kP)) { mPID.setP(p); kP = p; }
        // if((i != kI)) { mPID.setI(i); kI = i; }
        // if((d != kD)) { mPID.setD(d); kD = d; }
        previousYaw = currentYaw;
        currentYaw = s_Swerve.getGyroYaw().getDegrees();
        while (currentYaw > 360 || currentYaw < -360){
            currentYaw = currentYaw - 360;
        }

        
        //currentYaw =
        //SmartDashboard.putString("yaw acceleration", s_Swerve.gyro.getAngularVelocityYWorld().toString());

        previousRotation = commandedRotationVal;

        

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        commandedRotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (commandedRotationVal == 0 && previousRotation != 0){
            goalYaw = currentYaw;
        }

        SmartDashboard.putNumber("yaw", currentYaw);
        

        if (commandedRotationVal == 0 && (strafeVal != 0 || translationVal != 0)){
            rotationCorrect = mPID.getOutput(currentYaw, goalYaw);
        }

        SmartDashboard.putNumber("rotation goal", goalYaw);
        SmartDashboard.putNumber("PID output", rotationCorrect);

        rotationVal = 0;

        if (commandedRotationVal != 0){
            rotationVal = commandedRotationVal;
        }
        else if (translationVal != 0 || strafeVal != 0){
            if (rotationCorrect > 0.1){
                rotationCorrect = 0.1;
            }
            else if (rotationCorrect < -0.1){
                rotationCorrect = -0.1; 
            }
            rotationVal = rotationCorrect;
        }
        SmartDashboard.putNumber("turning", rotationVal);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

        
    }
}