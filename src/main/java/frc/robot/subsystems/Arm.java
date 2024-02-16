package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  //private DoubleSolenoid pistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.WRIST_FORWARD, Constants.WRIST_REVERSE);

  private final TalonFX mLeader = new TalonFX(32, "CTREDevicesCanivore");
  private final TalonFX mFollower = new TalonFX(31, "CTREDevicesCanivore");
  public Arm() {
    //mLeader.getConfigurator().apply(new TalonFXConfiguration());
    //mFollower.getConfigurator().apply(new TalonFXConfiguration());

    mLeader.setInverted(true);
    mFollower.setInverted(false);

    mLeader.setNeutralMode(NeutralModeValue.Brake);
    mFollower.setNeutralMode(NeutralModeValue.Brake);

    mFollower.setControl(new StrictFollower(mLeader.getDeviceID()));

    
  }

  @Override
  public void periodic() {
  }

  public void MoveArm(double speed) {
    // if (up) {
    //   pistons.set(Value.kReverse);
    // } else {
    //   pistons.set(Value.kForward);
    // }
    mLeader.set(speed);
  }
}
