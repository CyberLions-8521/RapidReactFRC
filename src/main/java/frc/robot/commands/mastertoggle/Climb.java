package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.dreadsubsystem.Climber;

public class Climb extends CommandBase {
  private final Climber m_climber;

  public Climb(Climber climber) {
    // set m_subsystem equal to whatever you put in Test
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_climber.initializeEncoder();
  }

  @Override
  public void execute() {
    m_climber.tarzan(RobotContainer.m_controller);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}