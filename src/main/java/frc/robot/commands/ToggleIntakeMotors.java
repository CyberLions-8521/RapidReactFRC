package frc.robot.commands;

import frc.robot.subsystems.ToggleSystems.ToggleGeneralMotors;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeMotors extends CommandBase {
    private final ToggleGeneralMotors m_motorsIntake;
    private boolean m_isDone;

    public ToggleIntakeMotors(ToggleGeneralMotors subsystem) {
        m_motorsIntake = subsystem;
        m_isDone = false;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (m_motorsIntake.getIntakeStatus() == false) {
            m_motorsIntake.IntakeOn();
        } else {
            m_motorsIntake.IntakeOff();
        }
        m_isDone = true;
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
