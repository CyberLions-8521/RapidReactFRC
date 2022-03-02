package frc.robot.subsystems.pneumatics;

// Pneumatic Dependecies (API)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SubsystemConstants;


public class SolenoidsIntakeSystem extends SubsystemBase {
    DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    DoubleSolenoid m_transLeftDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
    DoubleSolenoid m_transRightDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.CAN.kIntake, MotorType.kBrushed); // 775 Intake


    boolean ArmStatus;
    boolean GearStatus;
    public SolenoidsIntakeSystem() {
        retractArms();
        setGear1();
    }
    public void setSpeed(double speed) {
        m_IntakeMotor.set(speed);
    }

    public void extendArms() {
        m_leftArmDS.set(kForward);
        m_rightArmDS.set(kForward);
        m_IntakeMotor.set(0.55);
        ArmStatus = true;
    }

    public void retractArms() {
        m_leftArmDS.set(kReverse);
        m_rightArmDS.set(kReverse);
        m_IntakeMotor.set(0.0);
        ArmStatus = false;
    }

    public void disableArms() {
        m_leftArmDS.set(kOff);
        m_rightArmDS.set(kOff);
    }

    public void setGear1() {
        m_transLeftDS.set(kForward);
        m_transRightDS.set(kForward);
        GearStatus = true;
    }

    public void setGear2() {
        m_transLeftDS.set(kReverse);
        m_transRightDS.set(kReverse);
        GearStatus = false;
    }

    // public Value getArmStatus() {
    //     return m_rightArmDS.get();
    // }

    public boolean getArmStatus() {
        return ArmStatus;
    }

    public boolean getGearStatus() {
        return GearStatus;
    }

    // public int getGearStatus() {
    //     if (m_transRightDS.get().equals(kForward)) {
    //         return 1;
    //     } else {
    //         return 2;
    //     }
    // }

    

    @Override
    public void periodic() {
        //TODO: thien fix later
        SmartDashboard.putBoolean("Arm", getArmStatus()); 
        //SmartDashboard.putNumber("Gear", getGearStatus()); ///Timmy fix this too boolean later pls
        SmartDashboard.putBoolean("Gear Status", getGearStatus());
    }
}
