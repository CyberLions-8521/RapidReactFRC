package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSystem extends SubsystemBase {
    Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    
    public CompressorSystem() {
        disable();
    }

    public boolean enabled() {
        return m_compressor.enabled();
    }

    public void disable() {
        m_compressor.disable();
    }

    public void enable() {
        m_compressor.enableDigital();
    }
}
