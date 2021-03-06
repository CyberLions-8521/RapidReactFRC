// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.XBOX;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.utilsubsystem.ExampleSubsystem;
import frc.robot.subsystems.utilsubsystem.Limelight;
import frc.robot.utility.KevinLib;
import java.util.Arrays;
import java.util.List;

/** An example command that uses an example subsystem. */
public class LimeLightAimAssist extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Limelight m_vision;
  private final Drivebase m_drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimeLightAimAssist(Limelight vision, Drivebase drive) {
    m_vision = vision;
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }





  

  // public void AimAssistWithDriveBase(){
  //   double xOffset = m_vision.getTx();
  //   if (RobotContainer.m_controller.getRawButton(XBOX.X)){
  //   double output = PIDTurn.calculate(xOffset, 0.1);
  //   m_drive.turnInPlace(output);
  //   } else if(RobotContainer.m_controller.getRawButton(XBOX.X) == false){ 
  //   m_drive.arcadeDrive(RobotContainer.m_controller);
  //   }


  // }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(true){
    m_drive.turnInPlace(m_vision.AimAssist());
    }
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
