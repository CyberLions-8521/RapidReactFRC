package frc.robot.commands.autonomous;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.IndexSubsystem;
// import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLowerindex extends CommandBase {
  IndexSubsystem m_arm;

  public AutoLowerindex(IndexSubsystem subsystem) {
    m_arm = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_arm.lowerIndexOn();

    // while (true) {
    //   m_arm.lowerIndexOn();
    // }
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