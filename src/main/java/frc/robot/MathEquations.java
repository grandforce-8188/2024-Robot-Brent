package frc.robot;

public class MathEquations {
    public double getShootingArmAngle(double distance){
        // double angle = Math.pow(distance, 3) * 0.6139;
        // angle = angle - (Math.pow(distance, 2) * 6.9305);
        // angle = angle + (distance * 25.589);
        // angle = angle - 11.39;
        // angle = angle - 8;
        double angle = Math.pow(distance, 4) * -0.9031;
        angle = angle + (Math.pow(distance, 3) * 12.93);
        angle = angle - (Math.pow(distance, 2) * 68.4);
        angle = angle + (distance * 165.51);
        angle = angle - 123.03;
        return angle;
    }

    public double getShootingSpeed(double distance){
        double speed = Math.log(distance) * 40.241;
        speed = speed + 31.086;
        return speed;
    }
}
