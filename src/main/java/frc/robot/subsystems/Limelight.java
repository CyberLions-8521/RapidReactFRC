// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {

/** Creates a new ExampleSubsystem. */
public Limelight() {
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
  double z = getpipe.getDouble(0.0);
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  boolean TargetDetected =tv.getDouble(0.0)==1;
  // post to smart dashboard periodically
  SmartDashboard.putNumber("Pipeline", z);
  SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
  SmartDashboard.putBoolean("Target Detected", TargetDetected);
  SmartDashboard.putNumber("Distance", getDistanceToHub());
  

}

  //Filters
  
  LinearFilter yFilter = LinearFilter.movingAverage(10);

 public double getDistanceToHub(){
  double yOffset = Math.toRadians(yFilter.calculate(getTy()));
  double Distance = ((VisionConstants.HeightOfTarget - VisionConstants.HeightOfCamera) / Math.tan(VisionConstants.CameraAngle+yOffset));

  return Distance;
 }

 public double DistanceToMotorVelocity(){
  //insert hao's math
  double motorVelocity=0;
  return motorVelocity;

 }

  public double getTy(){
    return ty.getDouble(0.0);


  
}

public double getTx(){
  return ty.getDouble(0.0);



}

public double getTa(){
return ta.getDouble(0.0);




}

public boolean getHasTarget(){
return tv.getDouble(0.0)==1;



}
// public NetworkTableEntry getTy() {
  // return ty;
//}


@Override
public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
}


}