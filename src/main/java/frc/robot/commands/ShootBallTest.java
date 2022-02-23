
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//imports 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.KevinShooter;
import frc.robot.RobotContainer;

public class ShootBallTest extends CommandBase {
    /** Creates a new Drive. */
    private final KevinShooter m_Shoot;
    public ShootBallTest(KevinShooter shoot) {
      m_Shoot = shoot;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(shoot);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
      //for testing
      m_Shoot.Shooter(2000);
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
  