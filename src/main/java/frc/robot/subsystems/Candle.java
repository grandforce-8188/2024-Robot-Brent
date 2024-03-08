// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.networktables.*;

// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdleConfiguration;

// import frc.robot.Constants;

// import edu.wpi.first.wpilibj2.command.Command;



// public class Candle extends SubsystemBase {
//   //private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.FLIPPER_FORWARD, Constants.FLIPPER_REVERSE);
//   //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

//   final CANdle mCandle = new CANdle(Constants.CANdleID);
//   final CANdleConfiguration config = new CANdleConfiguration();

//   public Candle() {
    

//     config.stripType = LEDStripType.RGB; // set the strip type to RGB

//     config.brightnessScalar = 1; // dim the LEDs to half brightness


//     mCandle.configAllSettings(config);
//   }

//   @Override
//   public void periodic() {
//   }
//   public void setColor(String color){
//     if (color == "RED"){
//       mCandle.setLEDs(255, 0, 0);
//     }
//     else if (color == "BLUE"){
//       mCandle.setLEDs(0, 0, 255);
//     }
//     else if (color == "GREEN"){
//       mCandle.setLEDs(0, 255, 0);
//     }
//     else if (color == "YELLOW"){
//       mCandle.setLEDs(255, 255, 0);
//     }
//   }
//   public Command setStationColor(){
//     var alliance = DriverStation.getAlliance();
//     if (alliance.get() == Alliance.Red){
//       return this.runOnce(() -> setColor("RED"));
//     }
//     else{
//       return this.runOnce(() -> setColor("BLUE"));
//     }
//   }
// }
