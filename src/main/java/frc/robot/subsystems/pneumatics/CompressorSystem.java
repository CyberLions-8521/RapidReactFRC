package frc.robot.subsystems.pneumatics;

// Pneumatic Dependencies (API)
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSystem extends SubsystemBase {
    Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    
    public CompressorSystem() {}

    public boolean enabled() {
        return m_compressor.enabled();
    }

    public void disable() {
        m_compressor.disable();
    }

    public void enable() {
        m_compressor.enableDigital();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Compressor On", enabled());
    }
}
