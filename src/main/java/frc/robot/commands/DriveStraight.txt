// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.XBOX;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class DriveStraight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivebase m_Db;
  public AHRS m_Gyro;
  public double m_Speed;
  double m_InitHeading;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraight(Drivebase db) {
    m_Db = db;
    m_Gyro=db.getGyro();
    m_Speed=db.speed;
    double m_InitHeading;


    addRequirements(db);

  }

  double kP=69;
  double kI=69;
  double kD=69;
  PIDController pid = new PIDController(kP, kI, kD);


  

  @Override
  public void initialize() {
   // m_Gyro.reset();
    m_InitHeading = m_Gyro.getAngle();

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double output=pid.calculate(m_InitHeading, 0);

     m_Db.moveForward(-m_Speed, output);

  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
