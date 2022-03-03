package frc.robot.commands.dstoggle;

import frc.robot.subsystems.pneumatics.SolenoidsSystem;
import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ToggleIntakeSystem extends CommandBase {
  private final SolenoidsSystem m_solenoids;
  private final ToggleGeneralMotors m_intakeMotor;
  private boolean m_isDone;

  public ToggleIntakeSystem(SolenoidsSystem subsystem, ToggleGeneralMotors subMotor) {
    m_solenoids = subsystem;
    m_intakeMotor = subMotor;
    m_isDone = false;
    addRequirements(subsystem);
    addRequirements(subMotor);
  }

  @Override
  public void initialize() {
    if (m_solenoids.getArmStatus().equals(kOff)
        || m_solenoids.getArmStatus().equals(kReverse) && m_intakeMotor.getIntakeStatus() == false) {
      m_intakeMotor.IntakeOn();
      m_solenoids.extendArms();
    } else {
      m_solenoids.retractArms();
      m_intakeMotor.IntakeOff();
    }
    m_isDone = true;
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
