// package frc.robot.commands;

// import frc.robot.subsystems.ToggleSystems.ToggleGeneralMotors;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class ToggleLowerIndex extends CommandBase {
//   private final ToggleGeneralMotors m_motorsLower;
//   private boolean m_isDone;

//   public ToggleLowerIndex(ToggleGeneralMotors subsystem) {
//     m_motorsLower = subsystem;
//     m_isDone = false;
//     addRequirements(subsystem);
//   }

//   @Override
//   public void initialize() {
//     if (m_motorsLower.getLowIndexStatus()) {
//       m_motorsLower.LowerIndexOn();
//     } else {
//       m_motorsLower.LowerIndexOff();
//     }
//     m_isDone = true;
//   }

//   @Override
//   public boolean isFinished() {
//     return m_isDone;
//   }
// }
