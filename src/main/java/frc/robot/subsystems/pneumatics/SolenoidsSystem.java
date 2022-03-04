package frc.robot.subsystems.pneumatics;

// Pneumatic Dependecies (API)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import frc.robot.Constants.CAN;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidsSystem extends SubsystemBase {

    DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    DoubleSolenoid m_transLeftDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
    DoubleSolenoid m_transRightDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    CANSparkMax m_intake = new CANSparkMax(CAN.kIntake, MotorType.kBrushed); //Alternative autonomous mode

    boolean m_AutoStatus;


    public SolenoidsSystem() {
        retractArms();
        setGear1();
    }


    public void setSpeed(double speed) {
        m_intake.set(speed);
    }

    public void extendArms() {
        m_leftArmDS.set(kForward);
        m_rightArmDS.set(kForward);
    }


    public void AutoIntakeSystemON() {
        m_leftArmDS.set(kForward);
        m_rightArmDS.set(kForward);
        m_intake.set(0.5);
        m_AutoStatus = true;
    }

    public void AutoIntakeSystemOFF() {
        m_leftArmDS.set(kReverse);
        m_rightArmDS.set(kReverse);
        m_intake.set(0.5);
        m_AutoStatus = false;
    }

    public void retractArms() {
        m_leftArmDS.set(kReverse);
        m_rightArmDS.set(kReverse);
    }

    public void disableArms() {
        m_leftArmDS.set(kOff);
        m_rightArmDS.set(kOff);
    }

    public void setGear1() {
        m_transLeftDS.set(kForward);
        m_transRightDS.set(kForward);
    }

    public void setGear2() {
        m_transLeftDS.set(kReverse);
        m_transRightDS.set(kReverse);
    }

    public Value getArmStatus() {
        return m_rightArmDS.get();
    }

    public boolean getAutoStatus() {
        return m_AutoStatus;
    }

    public int getGearStatus() {
        if (m_transRightDS.get().equals(kForward)) {
            return 1;
        } else {
            return 2;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm", getArmStatus().toString());
        SmartDashboard.putNumber("Gear", getGearStatus());
    }
}
