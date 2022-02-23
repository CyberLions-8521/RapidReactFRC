// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.XBOX;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.KevinShooter;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class ShootBallToggle extends CommandBase {
  //ignore problems
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // initlalize all instant fields (variables in a class) outside 
  private boolean shot;
  //create shooter object
  private final KevinShooter m_shoot;

  //takes the aforementioned, newly created object and assigns a value to it
  public ShootBallToggle(KevinShooter subsystem) {
    //sets our value to the value wihtin the object
    //m_shoot is equal to wahtever subsystem is (subsystem is a placeholder )
    m_shoot = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    //gets values from Shooter.java and sets it equal to the aforementioned, newly created object
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //set shoot to button...
  //if button press, set rpm & shoot
  public void execute() {
    if (RobotContainer.m_controller.getRawButtonPressed(XBOX.A)) {
      m_shoot.maxRPM = 10000;
    } else if (RobotContainer.m_controller.getRawButtonReleased(XBOX.A)) {
      m_shoot.maxRPM = 0;
    } else {
      m_shoot.maxRPM = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //idk yet
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}