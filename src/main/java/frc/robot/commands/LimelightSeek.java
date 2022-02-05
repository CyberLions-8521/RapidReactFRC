// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.KevinLimelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/**
 *  This class finds a power cell and then moves close to it.
 */
public class LimelightSeek extends CommandBase {
  /** Creates a new LimelightSeek. */
  KevinLimelight m_cam;
  Drivebase m_db;
  boolean m_targetFound;
  boolean m_targetReached;
  double m_steeringAdjust;
  

  public LimelightSeek(Drivebase db, KevinLimelight cam)
  {
    m_cam = cam;
    m_db = db;
    m_steeringAdjust =0;
    m_targetFound = m_cam.getHasTarget();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(db);
    addRequirements(cam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    searchForTarget();

    if (m_targetFound && Math.abs(m_cam.getTx()) < DriveConstants.STEER_THRESHOLD)
    {
      moveToTarget();
    }
    
    // Since m_steeringAdjust isn't ever TRULY zero, we choose a number close enough to 0
    // when the robot stops to determine if we found the ball and centered ourselves onto it
    
    // If we found the target and we're centered on it
    
  }

  /**
   * Uses limelight output data to search for the target
   * If it doesn't detect any balls in sight, it'll spin clockwise to search for it
   * If it sees a ball, it'll turn and center itself onto it
   */
  public void searchForTarget()
  {
    // Check if the target is in the view
    m_targetFound = m_cam.getHasTarget();
    // Get the offset from the center of the camera and the target
    double offset = -m_cam.getTx();
    m_steeringAdjust = 0.0;
    // If we do not see the target, adjust the steering
    if (m_targetFound)
    {
      m_steeringAdjust = 0.1;
    }
    // We DO see the target
    else
    {
      m_steeringAdjust = Math.min(Math.abs(DriveConstants.STEER_K * offset), 5);
    }
    // Reapply the sign
    m_steeringAdjust = Math.copySign(m_steeringAdjust, offset);
    // Reverse the sign?
    m_db.turnInPlace(m_steeringAdjust);

    // Update the value on the dashboard
    SmartDashboard.putNumber("Steering Adjust", m_steeringAdjust);

  }

  /**
   * Moves toward a target that the limelight detects by looking at the area of the ball relative to the screen
   */
  public void moveToTarget()
  {
    // Speed at which the robot moves toward the ball
    double speed = 10;
    // The amount of space the ball takes up in the camera view
    double area = m_cam.getTa();  
    if (area < VisionConstants.BALL_AREA)
    {
      // Then move towards it slowly
      speed = -DriveConstants.DRIVE_SLOW;
      m_targetReached = false;
    }
    else
    {
      speed = 0;
      m_targetReached = true;
    }

    m_db.moveForward(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop after you reach a target and it's been 5 seconds
    // return m_targetReached;
    return false;
  }

  public void setAutonomousCommand() {
  }
}