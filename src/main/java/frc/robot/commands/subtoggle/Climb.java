package frc.robot.commands.subtoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  private final Climber m_climber;

  public Climb(Climber m_climber) {
    // set m_subsystem equal to whatever you put in Test
    this.m_climber = m_climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
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