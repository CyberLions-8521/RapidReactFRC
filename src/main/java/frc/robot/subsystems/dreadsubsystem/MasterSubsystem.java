package frc.robot.subsystems.dreadsubsystem;

// Pneumatic Dependecies (API)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class MasterSubsystem extends SubsystemBase {

    DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    DoubleSolenoid m_transLeftDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
    DoubleSolenoid m_transRightDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    CANSparkMax m_intake = new CANSparkMax(CAN.kIntake, MotorType.kBrushed); 
    CANSparkMax m_FrontIndexor = new CANSparkMax(Constants.CAN.kIndexorFront, MotorType.kBrushless);
    CANSparkMax m_BackIndexor = new CANSparkMax(Constants.CAN.kIndexorBack, MotorType.kBrushed);
    CANSparkMax m_LowIndexor = new CANSparkMax(Constants.CAN.kIndexorLower, MotorType.kBrushless);
    
    boolean m_AutoStatus;
    boolean m_lowindexorStatus;
    boolean m_indexorStatus;
    boolean m_intakeStatus;
    boolean m_toggleSystemStatus;



    public MasterSubsystem() {
        // stop all subsystem toggle here
       
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

    // Double solenoid transmission gear

    public void setGear1() {
        m_transLeftDS.set(kForward);
        m_transRightDS.set(kForward);
    }

    public void setGear2() {
        m_transLeftDS.set(kReverse);
        m_transRightDS.set(kReverse);
    }

    // boolean status check

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

    

    public void setMotor(double speed) {
        m_FrontIndexor.set(speed);
        m_BackIndexor.set(speed);
        m_LowIndexor.set(speed);
        m_intake.set(speed);
    }

    public boolean getIndexStatus() {
        return m_indexorStatus;
    }

    public boolean getLowerIndexStatus() {
        return m_lowindexorStatus;
    }

    public boolean getIntakeStatus() {
        return m_intakeStatus;
    }

    public void IndexOn() {
        m_FrontIndexor.set(0.60);
        m_BackIndexor.set(0.50);

        m_indexorStatus = true;
    }

    public void IndexOff() {
        m_FrontIndexor.set(0.0);
        m_BackIndexor.set(0.0);
        m_indexorStatus = false;
    }

    public void lowerIndexON() {
        m_LowIndexor.set(0.6);
        m_lowindexorStatus = true;
    }

    public void lowerIndexOff() {
        m_LowIndexor.set(0.0);
        m_lowindexorStatus = false;
    }

    public void IntakeOn() {
        m_intake.set(0.6);
        m_intakeStatus = true;

    }

    public void IntakeOff() {
        m_intake.set(0.0);
        m_intakeStatus = false;
    }


    public boolean getSystemStatus() {
        return m_toggleSystemStatus;
    }



    @Override
    public void periodic() {
        SmartDashboard.putString("Arm", getArmStatus().toString());
        SmartDashboard.putNumber("Gear", getGearStatus());
        SmartDashboard.putBoolean("Upper Index Status", getIndexStatus());
        SmartDashboard.putBoolean("Lower Index Status", getLowerIndexStatus());
        SmartDashboard.putBoolean("Intake Status", getIntakeStatus());
        SmartDashboard.putBoolean("Auto Intake/Index Status", getSystemStatus());
    }
    

}



