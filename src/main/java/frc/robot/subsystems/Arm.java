package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.GoalEndState;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.MiniPID;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
  //private DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.WRIST_FORWARD, Constants.WRIST_REVERSE);

  private final TalonFX mLeader = new TalonFX(Constants.armLeaderID, "CTREDevicesCanivore");
  private final TalonFX mFollower = new TalonFX(Constants.armFollowerID, "CTREDevicesCanivore");

  private final Slot0Configs slot0configs = new Slot0Configs();

  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

  private final OpenLoopRampsConfigs rampRate = new OpenLoopRampsConfigs();

  private final DutyCycleOut turnDutyCycle = new DutyCycleOut(0);

  double PIDSpeed = 0;

  private final CANcoder mEncoder = new CANcoder(Constants.CANcoderID, "CTREDevicesCanivore"); //MAKE SURE THIS IS CORRECT


  

  double kP = 0.07; 
  double kI = 0;
  double kD = 0;

  private final MiniPID mPID = new MiniPID(kP, kI, kD);

  
  public Arm() {
    //mLeader.getConfigurator().apply(new TalonFXConfiguration());
    //mFollower.getConfigurator().apply(new TalonFXConfiguration());

    // mTopArmSensor = BottomMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // mBottomArmSensor = TopMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    currentLimits.withSupplyCurrentLimit(Constants.currentLimit);
    currentLimits.withSupplyCurrentLimitEnable(true);
    rampRate.withDutyCycleOpenLoopRampPeriod(0.2);    

    mLeader.getConfigurator().apply(currentLimits);
    mLeader.getConfigurator().apply(rampRate);

    mFollower.getConfigurator().apply(currentLimits);
    mFollower.getConfigurator().apply(rampRate);

    mLeader.setInverted(false);
    mFollower.setInverted(true);

    mLeader.setNeutralMode(NeutralModeValue.Brake);
    mFollower.setNeutralMode(NeutralModeValue.Brake);




    mFollower.setControl(new StrictFollower(mLeader.getDeviceID()));
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
     
    //mLeader.setControl

    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("upper limit switch", GetTopLimitSwitch());
    SmartDashboard.putBoolean("lower limit switch", GetBottomLimitSwitch());
  }

  public void ManualMoveArm(double speed) {
    // if (up) {
    //   pistons.set(Value.kReverse);
    // } else {
    //   pistons.set(Value.kForward);
    // }
    

    
    speed = MathUtil.applyDeadband(speed, 0.05);
    
    speed = Math.pow(speed, 3);
    
    //double curveValue = 1;

    double lowerError = Constants.lowerArmPosLimit - GetEncoderPosition();
    double upperError = Constants.upperArmPosLimit - GetEncoderPosition();

    double lowerErrorSpeed = lowerError * 0.054;
    double upperErrorSpeed = upperError * 0.054;

    SmartDashboard.putNumber("Encoder position", GetEncoderPosition());

    // SmartDashboard.putNumber("lowerError", lowerError);
    // SmartDashboard.putNumber("upperError", upperError);
    // SmartDashboard.putNumber("lowerErrorSpeed", lowerErrorSpeed);
    // SmartDashboard.putNumber("upperErrorSpeed", upperErrorSpeed);

    if (speed < 0){
      if (speed < lowerErrorSpeed){
        speed = lowerErrorSpeed;
      }
    }

    else if (speed > 0){
      if (speed > upperErrorSpeed){
        speed = upperErrorSpeed;
      }
    }   

    if (speed < 0 && !GetBottomLimitSwitch()){
      speed = 0;
    }

    if (speed > 0 && !GetTopLimitSwitch()){
      speed = 0;
    }

    SmartDashboard.putNumber("CommandedArmSpeed", speed);

    if (speed > Constants.maxManualArmSpeed){
      speed = Constants.maxManualArmSpeed;
    }
    if (speed < -Constants.maxManualArmSpeed){
      speed = -Constants.maxManualArmSpeed;
    }

    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
  
    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { mPID.setP(p); kP = p; }
    // if((i != kI)) { mPID.setI(i); kI = i; }
    // if((d != kD)) { mPID.setD(d); kD = d; }

    turnDutyCycle.Output = speed;
    turnDutyCycle.EnableFOC = true;
    mLeader.setControl(turnDutyCycle);

    //mLeader.set(speed);

  }

  public void PIDMoveArm(double position){
    if (position > Constants.upperArmPosLimit){
      position = Constants.upperArmPosLimit;
    }
    if (position < Constants.lowerArmPosLimit){
      position = Constants.lowerArmPosLimit;
    }
    PIDSpeed = 0;
    PIDSpeed = mPID.getOutput(GetEncoderPosition(), position);
    SmartDashboard.putNumber("PID commanded speed", PIDSpeed);
    SmartDashboard.putNumber("Commanded position", position);
    //System.out.println(PIDSpeed);

    if (PIDSpeed > 1){
      PIDSpeed = 1;
    }
    if (PIDSpeed < -1){
      PIDSpeed = -1;
    }

    

    double lowerError = Constants.lowerArmPosLimit - GetEncoderPosition();
    double upperError = Constants.upperArmPosLimit - GetEncoderPosition();

    double lowerErrorSpeed = lowerError * 0.054;
    double upperErrorSpeed = upperError * 0.054;

    if (PIDSpeed < 0){
      if (PIDSpeed < lowerErrorSpeed){
        PIDSpeed = lowerErrorSpeed;
      }
    }

    else if (PIDSpeed > 0){
      if (PIDSpeed > upperErrorSpeed){
        PIDSpeed = upperErrorSpeed;
      }
    }   


    if (PIDSpeed < 0 && !GetBottomLimitSwitch()){
      PIDSpeed = 0;
    }

    if (PIDSpeed > 0 && !GetTopLimitSwitch()){
      PIDSpeed = 0;
    }

    // if (PIDSpeed > 0.2){
    //   PIDSpeed = 0.2;
    // }
    // if (PIDSpeed < -0.2){
    //   PIDSpeed = -0.2;
    // }
    mLeader.set(PIDSpeed);

  }

  public double GetEncoderPosition(){
    // position = mEncoder.getAbsolutePosition();
    var position = mEncoder.getAbsolutePosition();
    double adjustedPos = (position.getValueAsDouble() * 360.0) + 19.27;
    SmartDashboard.putNumber("arm position", adjustedPos);
    
    return adjustedPos;
  }

  public boolean IsPIDFinished(double goalState){
    
    if (Math.abs(GetEncoderPosition() - goalState) < 1){
      return true;
    }
    return false;
  }

  public boolean GetTopLimitSwitch(){
    return toplimitSwitch.get();
  }

  public boolean GetBottomLimitSwitch(){
    return bottomlimitSwitch.get();
  }
}
