package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class toggleReverseIndexSystem extends CommandBase {
  private final MasterSubsystem m_reverseIndex;

  public toggleReverseIndexSystem(MasterSubsystem subsystem) {
    m_reverseIndex = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  //   if (m_reverseIndex.getReverseStatus() == false) {
  //     m_reverseIndex.reverseIndexOn();

  //   } else {
  //     m_reverseIndex.reverseIndexOff();
  //   }
  //   m_returnStatus = true;
  // } 
  } 


  @Override
  public void execute() {
  //  m_reverseIndex.ToggleReverseIndex(RobotContainer.m_controller);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
