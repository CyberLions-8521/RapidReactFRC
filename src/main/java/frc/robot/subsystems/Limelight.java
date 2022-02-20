
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry getpipe = table.getEntry("getpipe");

  // Math calculation
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

  // public NetworkTableEntry getTy() {
  // return ty;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read values periodically
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}