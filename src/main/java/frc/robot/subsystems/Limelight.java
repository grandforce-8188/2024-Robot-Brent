package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.*;


public class Limelight extends SubsystemBase {
  //private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.FLIPPER_FORWARD, Constants.FLIPPER_REVERSE);
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public Limelight() {
  }

  @Override
  public void periodic() {
  }

  public double getVisibleAprilTags() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(10);
  }
  public double getSpeakerDistance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == Alliance.Red){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }
    else if (alliance.get() == Alliance.Blue){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    }
    //double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDouble(0);
    // double[] pos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    // //System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]));
    // for (double element : pos) {
    //   System.out.println(element);
    // }\

    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    SmartDashboard.putNumberArray("botpose", botpose);
    SmartDashboard.putNumber("direction0", botpose[0]);
    SmartDashboard.putNumber("direction1", botpose[1]);
    SmartDashboard.putNumber("direction2, wanted value", botpose[2]);
    SmartDashboard.putNumber("direction3", botpose[3]);
    SmartDashboard.putNumber("direction4", botpose[4]);
    SmartDashboard.putNumber("direction5", botpose[5]);
    double tx = botpose[2];

    return tx;
  }
  public double getSpeakerCenter(){
    var alliance = DriverStation.getAlliance();
    double tx = 0;
    if (alliance.get() == Alliance.Red){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    //double tz = 0.0;
      tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }
    else if (alliance.get() == Alliance.Blue){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    //double tz = 0.0;
      tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    //SmartDashboard.putNumber("botpose", botpose);
    
    return tx;
  }
}
