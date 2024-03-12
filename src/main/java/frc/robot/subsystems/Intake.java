package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

 // private final CANSparkMax mIntake = new CANSparkMax(Constants.intakeID, MotorType.kBrushed);

  private final TalonFX mIntake = new TalonFX(Constants.intakeID, "rio");

  private final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();

  private final OpenLoopRampsConfigs rampRate = new OpenLoopRampsConfigs();

  public Intake() {

    // mIntake.setIdleMode(IdleMode.kBrake);
    // mIntake.setSmartCurrentLimit(Constants.currentLimit);

    mIntake.setNeutralMode(NeutralModeValue.Coast);

    currentLimits.withSupplyCurrentLimit(Constants.currentLimit);
    currentLimits.withSupplyCurrentLimitEnable(true);
    currentLimits.withStatorCurrentLimit(Constants.currentLimit);
    currentLimits.withStatorCurrentLimitEnable(true);
    rampRate.withDutyCycleOpenLoopRampPeriod(0.2); 

    mIntake.getConfigurator().apply(currentLimits);
    mIntake.getConfigurator().apply(rampRate);
    
  }

  @Override
  public void periodic() {
  }

  public void RunIntake(double speed) {
    mIntake.set(speed);
    
  }
}
