package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class SolenoidsSystem extends SubsystemBase {
    DoubleSolenoid m_leftDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    DoubleSolenoid m_rightDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
    public SolenoidsSystem() {
        disableArms();
    }

    public void extendArms() {
        m_leftDS.set(kForward);
        m_rightDS.set(kForward);
    }

    public void retractArms() {
        m_leftDS.set(kReverse);
        m_rightDS.set(kReverse);
    }

    public void disableArms() {
        m_leftDS.set(kOff);
        m_rightDS.set(kOff);
    }

    public Value get() {
        return m_rightDS.get();
    }

    @Override
    public void periodic() {
        //TODO: thien fix later
        SmartDashboard.putString("Solenoid Status", get().toString()); 
    }
}
