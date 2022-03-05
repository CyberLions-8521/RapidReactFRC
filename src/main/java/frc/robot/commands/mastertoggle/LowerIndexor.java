package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class LowerIndexor extends CommandBase {

    private final MasterSubsystem m_lowindexmotor;
    private boolean m_returnStatus;

    public LowerIndexor(MasterSubsystem subsystem) {
        m_lowindexmotor = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (m_lowindexmotor.getLowerIndexStatus() == false) {
            m_lowindexmotor.lowerIndexON();

        } else {
            m_lowindexmotor.IndexOff();
        }
        m_returnStatus = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_returnStatus;
    }
}
