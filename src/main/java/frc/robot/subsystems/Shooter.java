package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.MiniPID;

public class Shooter extends SubsystemBase {
  //private DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.WRIST_FORWARD, Constants.WRIST_REVERSE);

//   public final CANSparkFlex mTop = new CANSparkFlex(Constants.shooterTopID, MotorType.kBrushless);
//   public final CANSparkFlex mBottom = new CANSparkFlex(Constants.shooterBottomID, MotorType.kBrushless);

  public final TalonFX mTop = new TalonFX(Constants.shooterTopID, "rio");
  public final TalonFX mBottom = new TalonFX(Constants.shooterBottomID, "rio");

  double kP = 0.3; 
  double kI = 0;
  double kD = 1;

  public final MiniPID mPid = new MiniPID(kP, kI, kD);

  private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

  private final OpenLoopRampsConfigs rampRate = new OpenLoopRampsConfigs();


//   private final SparkLimitSwitch mTopArmSensor = mBottom.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
//   private final SparkLimitSwitch mBottomArmSensor = mTop.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
  
//   private final SparkPIDController mTopPID = mTop.getPIDController();
//   private final SparkPIDController mBottomPID = mBottom.getPIDController();


//   RelativeEncoder topEncoder = mTop.getVelocity;
//   RelativeEncoder bottomEncoder = mBottom.getEncoder();

  public Shooter() {
    //mLeader.getConfigurator().apply(new TalonFXConfiguration());
    //mFollower.getConfigurator().apply(new TalonFXConfiguration());

    mTop.setInverted(true);
    mBottom.setInverted(false);

    mTop.setNeutralMode(NeutralModeValue.Coast);
    mBottom.setNeutralMode(NeutralModeValue.Coast);


    currentLimits.withSupplyCurrentLimit(Constants.currentLimit);
    currentLimits.withSupplyCurrentLimitEnable(true);
    currentLimits.withStatorCurrentLimit(Constants.currentLimit);
    currentLimits.withStatorCurrentLimitEnable(true);
    rampRate.withDutyCycleOpenLoopRampPeriod(0.2);  

    mTop.getConfigurator().apply(currentLimits);
    mTop.getConfigurator().apply(rampRate);

    mBottom.getConfigurator().apply(currentLimits);
    mBottom.getConfigurator().apply(rampRate);

    mBottom.setControl(new StrictFollower(mTop.getDeviceID()));

    // mTop.setSmartCurrentLimit(Constants.currentLimit);
    // mBottom.setSmartCurrentLimit(Constants.currentLimit);

    // mTopPID.setP(1);
    // mTopPID.setI(0);
    // mTopPID.setD(0.0);
    // mTopPID.setIZone(0);
    // mTopPID.setFF(0.0001);
    // mTopPID.setOutputRange(-6000, 6000);

    // mBottomPID.setP(1);
    // mBottomPID.setI(0);
    // mBottomPID.setD(0.0);
    // mBottomPID.setIZone(0);
    // mBottomPID.setFF(0.0001);
    // mBottomPID.setOutputRange(-6000, 6000);

    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Shooter speed", 0);

    //mFollower.setControl(new StrictFollower(mLeader.getDeviceID()));

    
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Top RPM", getTopRPM());
    SmartDashboard.putNumber("Bottom RPM", getBottomRPM());
  }


  public double getTopRPM(){   
    return mTop.getVelocity().getValueAsDouble();
  }

  public double getBottomRPM(){
    return mBottom.getVelocity().getValueAsDouble();
  }

//   public boolean getTopArmSensor(){
//     return mTopArmSensor.isPressed();
//   }
//   public boolean getBottomArmSensor(){
//     return mBottomArmSensor.isPressed();
//   }

  // public void RunShooterPercent(double topSpeed, double bottomSpeed) {
  //   // if (up) {
  //   //   pistons.set(Value.kReverse);
  //   // } else {
  //   //   pistons.set(Value.kForward);
  //   // }
  //   mTop.set(topSpeed);
  //   mBottom.set(bottomSpeed);
    

    
  // }

  public void RunShooterRPM(double speed){
    // mTopPID.setReference(-topSpeed, CANSparkFlex.ControlType.kVelocity);
    // mBottomPID.setReference(bottomSpeed, CANSparkFlex.ControlType.kVelocity);

    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
  
    // //if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { mPid.setP(p); kP = p; }
    // if((i != kI)) { mPid.setI(i); kI = i; }
    // if((d != kD)) { mPid.setD(d); kD = d; }
    //double speed = SmartDashboard.getNumber("Shooter speed", 0);
    //SmartDashboard.putNumber("Shooter PID k", );

    
    SmartDashboard.putNumber("Shooter goal speed", speed);

    double TopSpeed = mPid.getOutput(getTopRPM(), speed);

    SmartDashboard.putNumber("Shooter commanded speed", TopSpeed);


    //double BottomSpeed = mPid.getOutput(getBottomRPM(), bottomSpeed);

    mTop.set(TopSpeed);
    // mBottom.set(BottomSpeed);

    //SmartDashboard.putNumber("shooter goal speed", TopSpeed);

    //SmartDashboard.putNumber("Top Goal", TopSpeed);
    //SmartDashboard.putNumber("Bottom Goal", bottomSpeed);

    // mTop.set(topSpeed);
    // mBottom.set(-bottomSpeed);
    //SmartDashboard.putData(getTopRPM());
    
  }

  public void StopShooter(){
    mTop.set(0);
    //mBottom.set(0);
  }
}
