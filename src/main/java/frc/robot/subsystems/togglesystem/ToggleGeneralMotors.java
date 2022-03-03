
package frc.robot.subsystems.togglesystem;

//imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Rev Robotics Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// Constants filter later
import frc.robot.Constants;

public class ToggleGeneralMotors extends SubsystemBase {
    boolean m_lowindexorStatus;
    boolean m_indexorStatus;
    boolean m_intakeStatus;

    CANSparkMax m_FrontIndexor = new CANSparkMax(Constants.CAN.kIndexorFront, MotorType.kBrushless);
    CANSparkMax m_BackIndexor = new CANSparkMax(Constants.CAN.kIndexorBack, MotorType.kBrushed);
    CANSparkMax m_LowIndexor = new CANSparkMax(Constants.CAN.kIndexorLower, MotorType.kBrushless);
    CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.CAN.kIntake, MotorType.kBrushed);

    public ToggleGeneralMotors() {
        IndexorOff();

    }

    public void setMotor(double speed) {
        m_FrontIndexor.set(speed);
        m_BackIndexor.set(speed);
        m_LowIndexor.set(speed);
        m_IntakeMotor.set(speed);
    }

    public boolean getIndexStatus() {
        return m_indexorStatus;
    }
    
    public boolean getIntakeStatus() {
        return m_intakeStatus;
    }

    public void IndexorOn() {
        m_FrontIndexor.set(0.60);
        m_BackIndexor.set(0.50);
        m_LowIndexor.set(0.60);
        m_indexorStatus = true;
    }

    public void IndexorOff() {
        m_FrontIndexor.set(0.0);
        m_BackIndexor.set(0.0);
        m_LowIndexor.set(0.0);
        m_indexorStatus = false;
    }

    public void IntakeOn() {
        m_IntakeMotor.set(0.6);
        m_intakeStatus = true;

    }

    public void IntakeOff() {
        m_IntakeMotor.set(0.0);
        m_intakeStatus = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexor Status", getIndexStatus());
        SmartDashboard.putBoolean("Intake Status", getIntakeStatus());

    }

}