// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PIDTurnToAngle extends CommandBase {
  private final Drivebase m_db;
  private final double m_Setpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   //Can be set in robot container to that degree
  public PIDTurnToAngle(Drivebase db, double setpoint) {
    m_db = db;
    m_Setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.getGyro().reset();
  }

  
  final double kP=0.025;
  final double kI=0;
  final double kD=0;
  //Proportional System Control
  //Set I,D to 0 to make it Proportional System
  //Proportinal equation output=measured-setpoint
  PIDController pid = new PIDController(kP, kI, kD);

  public void GyroTurn() {
    //Get Angle from Gyro
    double measuredAngle = m_db.getAngle();
    //Calculate output from PID api
    double output = pid.calculate(measuredAngle, m_Setpoint);
    //Put output from PID into drivesystem
    m_db.turnInPlace(output);

 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Runs method
    GyroTurn();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}