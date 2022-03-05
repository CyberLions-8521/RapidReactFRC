
package frc.robot.subsystems.togglesystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ToggleGeneralMotors extends SubsystemBase {
    boolean m_lowindexorStatus;
    boolean m_indexorStatus;
    boolean m_intakeStatus;

    CANSparkMax m_frontIndexor = new CANSparkMax(Constants.CAN.INDEXOR_FRONT, MotorType.kBrushless);
    CANSparkMax m_backIndexor = new CANSparkMax(Constants.CAN.INDEXOR_BACK, MotorType.kBrushed);
    CANSparkMax m_lowIndexor = new CANSparkMax(Constants.CAN.INDEXOR_LOWER, MotorType.kBrushless);
    CANSparkMax m_intakeMotor = new CANSparkMax(Constants.CAN.INTAKE, MotorType.kBrushed);

    public ToggleGeneralMotors() {
        indexOff();
        lowerIndexOff();
    }

    public void setMotor(double speed) {
        m_frontIndexor.set(speed);
        m_backIndexor.set(speed);
        m_lowIndexor.set(speed);
        m_intakeMotor.set(speed);
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
        m_frontIndexor.set(0.60);
        m_backIndexor.set(0.50);

        m_indexorStatus = true;
    }

    public void indexOff() {
        m_frontIndexor.set(0.0);
        m_backIndexor.set(0.0);
        m_indexorStatus = false;
    }

    public void lowerIndexOn() {
        m_lowIndexor.set(0.6);
        m_lowindexorStatus = true;
    }

    public void lowerIndexOff() {
        m_lowIndexor.set(0.0);
        m_lowindexorStatus = false;
    }

    public void intakeOn() {
        m_intakeMotor.set(0.6);
        m_intakeStatus = true;
    }

    public void intakeOff() {
        m_intakeMotor.set(0.0);
        m_intakeStatus = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Upper Index Status", getIndexStatus());
        SmartDashboard.putBoolean("Lower Index Status", getLowerIndexStatus());
        SmartDashboard.putBoolean("Intake Status", getIntakeStatus());
    }
}