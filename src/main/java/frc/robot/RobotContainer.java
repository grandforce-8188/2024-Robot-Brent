package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
//import frc.robot.commands.RunShooterPercent;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final CommandJoystick manipulatorJoystick = new CommandJoystick(1);

    private final Shooter mShooter = new Shooter();
    private final Intake mIntake = new Intake();
    private final Feeder mFeeder = new Feeder();
    private final Arm mArm = new Arm();
    //private final Candle mCandle = new Candle();
    private final Limelight mLimelight = new Limelight();

    UsbCamera cam0 = CameraServer.startAutomaticCapture(0);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis) * HeaidngCorrection(), 
                () -> -driver.getRawAxis(strafeAxis) * HeaidngCorrection(), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureAutoCommands();
        configureButtonBindings();

        this.autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putNumber("arm rotation", mArm.GetEncoderPosition());
        SmartDashboard.putData(mShooter);


        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

     private int HeaidngCorrection() {

        var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                                if(alliance.get() == DriverStation.Alliance.Red){
                                  return 1;
                                }else{
                                  return -1;
                                }
                            }
                            return -1;

        }
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        mArm.setDefaultCommand(new ManualArm(mArm, manipulatorJoystick));
        // mShooter.setDefaultCommand(new RunShooterRPM(mShooter, 0.0, 0.0));
        //mShooter.setDefaultCommand(new RunShooterRPM(mShooter, 0.0));
        mLimelight.setDefaultCommand(new getSpeakerDistance(mLimelight));


        mIntake.setDefaultCommand(new RunIntake(mIntake, 0));
        mFeeder.setDefaultCommand(new RunFeeder(mFeeder, 0.0));
    

        manipulatorJoystick.button(8).whileTrue(new RunShooterRPM(mShooter, 70.0));
        manipulatorJoystick.button(7).whileTrue(new RunShooterRPM(mShooter, -80.0));
        manipulatorJoystick.button(1).whileTrue(new TriggerPull(mArm, mShooter, mIntake, mFeeder, mLimelight, s_Swerve));

        manipulatorJoystick.button(2).whileTrue(new RunIntake(mIntake, -Constants.intakeSpeed));
        manipulatorJoystick.button(6).whileTrue(new RunIntake(mIntake, Constants.intakeSpeed));
        manipulatorJoystick.button(5).whileTrue(new RunIntake(mIntake, -Constants.intakeSpeed));

        manipulatorJoystick.button(3).whileTrue(new RunFeeder(mFeeder, -1.0));
        manipulatorJoystick.button(4).whileTrue(new RunFeeder(mFeeder, 1.0));


        manipulatorJoystick.button(11).whileTrue(new PIDArm(mArm, Constants.sourceArmPos));
        manipulatorJoystick.button(9).whileTrue(new PIDArm(mArm, Constants.lowerArmPosLimit));
        manipulatorJoystick.button(10).whileTrue(new ShootPreset(mArm, mShooter, mIntake, mFeeder, mLimelight, 70));
        manipulatorJoystick.button(12).whileTrue(new PIDArm(mArm, Constants.upperArmPosLimit));

    }

    public void configureAutoCommands(){
        NamedCommands.registerCommand("spinIntake", new RunIntake(mIntake, -Constants.intakeSpeed).withTimeout(1.5));
        NamedCommands.registerCommand("shootRPM", new ShootNoAlignSwerve(mArm, mShooter, mIntake, mFeeder, mLimelight, 80).withTimeout(1.5));
        NamedCommands.registerCommand("intakeDown", new PIDArm(mArm, Constants.lowerArmPosLimit).withTimeout(1));
        NamedCommands.registerCommand("slowShoot", new ShootPreset(mArm, mShooter, mIntake, mFeeder, mLimelight, 65).withTimeout(1.3));
        //NamedCommands.registerCommand("shoot", new Shoot(mShooter, mIntake, mFeeder).withTimeout(1));
        //NamedCommands.registerCommand("spinFeeder", new RunFeeder(mFeeder, -0.1).withTimeout(2));
        // NamedCommands.registerCommand("stopShooter", new RunShooterRPM(mShooter, 0));
        // NamedCommands.registerCommand("stopFeeder", new RunFeeder(mFeeder, 0));
        // NamedCommands.registerCommand("stopIntake", new RunIntake(mIntake, 0));


        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return autoChooser.getSelected();

    }
}
