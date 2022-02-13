package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.pneumatics.CompressorSystem;

public class ToggleCompressor extends CommandBase {
    private final CompressorSystem m_compressor;
    private boolean m_isDone = false;

    public ToggleCompressor(CompressorSystem subsystem) {
        m_compressor = subsystem;
        addRequirements(m_compressor);
    }

    @Override
    public void initialize() {
        if (m_compressor.enabled()) {
            m_compressor.disable();
        } else {
            m_compressor.enable();
        }
        m_isDone = true;
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
