// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.utilsubsystem;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    PortForwarder.add(5800, "10.85.21.103", 5800);
    PortForwarder.add(5801, "10.85.21.103", 5801);

    

  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry getpipe = table.getEntry("getpipe");

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read values periodically
    getDistanceToHub();
    double z = getpipe.getDouble(0.0);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    boolean TargetDetected = tv.getDouble(0.0) == 1;
    // post to smart dashboard periodically
    SmartDashboard.putNumber("Pipeline", z);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putBoolean("Target Detected", TargetDetected);
    SmartDashboard.putBoolean("In Range Of Hub", InRange());
    SmartDashboard.putNumber("Distance", getDistanceToHub());
  }

  // Filters

  public double getDistanceToHub() {
    LinearFilter yFilter =LinearFilter.movingAverage(10);
    double yOffset = Math.toRadians(yFilter.calculate(getTy()));
    double Distance = ((VisionConstants.TARGET_HEIGHT - VisionConstants.CAMERA_HEIGHT)
        / Math.tan(VisionConstants.CAMERA_ANGLE + yOffset));

    return Distance;
  }

  public boolean InRange(){
    boolean inRange;
    boolean TargetDetected = getHasTarget();
    double distance = getDistanceToHub();
    if (distance >= 7 && distance <= 14 && TargetDetected) {
      inRange = true;
    } else {
      inRange = false;
    }
 
    return inRange;
  }

  public double getDistanceToMotorVelocity() {
    double rad = .05088; // (meters)
    double grav = 9.8; // (m/s^2)

    // constant can be changed
    double heightDiff = (VisionConstants.TARGET_HEIGHT - VisionConstants.CAMERA_HEIGHT + .25);
    double angle = Math.toRadians(60); // adjustable
    double efficiency = 30;

    double motorVelocity = efficiency * ((grav * getDistanceToHub()) / (Math.cos(angle) * Math.sqrt(2 * grav * heightDiff)))
        * ((30/Math.PI)/rad);
        //9.8 x 4.2672  / ( sqrt(2 x 9.8 x 1.8) *cos pi/3) x pi/30 x .05088
    return motorVelocity;
  }

  public double getTy() {
    return ty.getDouble(0.0);
  }

  public double getTx() {
    return ty.getDouble(0.0);
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public boolean getHasTarget() {
    return tv.getDouble(0.0) == 1;
  }

  /*
   * public NetworkTableEntry getTy() {
   * return ty;
   * }
   */

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}