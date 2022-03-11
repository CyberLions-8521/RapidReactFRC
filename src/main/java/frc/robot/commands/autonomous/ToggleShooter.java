// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ToggleShooter extends CommandBase {
  private final Turret m_Turret;
 // private final MasterSubsystem m_index;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   //Can be set in robot container to that degree
  public ToggleShooter(Turret turret) {
    m_Turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
    

  }


/*
  public void AutoIndexerAuto(){
      //For testing
      /*

        /*  if(setpoint-50 <  m_encoder.getVelocity()){
             m_index.indexOn();
        } else {
        m_index.indexOff();
       }
      }*/
      
      
    /*  if(3970 < m_Turret.m_encoder.getVelocity()){
        m_index.indexOn();
      } else {
        m_index.indexOff();
      }
      
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }



 

 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Turret.setSpeed();
   //  AutoIndexerAuto();

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