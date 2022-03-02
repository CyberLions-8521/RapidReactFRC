package frc.robot.subsystems.togglesystems;

//imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotContainer;

//XBOX Controller Imports
import frc.robot.Constants.XBOX;
import edu.wpi.first.wpilibj.XboxController;

// Rev Robotics Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Constants filter later
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SubsystemConstants;

public class ToggleGeneralMotors extends SubsystemBase {
    boolean m_lowindexorStatus;
    boolean m_indexorStatus;
    boolean m_intakeStatus;

    CANSparkMax m_FrontIndexor = new CANSparkMax(Constants.CAN.kIndexorFront, MotorType.kBrushless); // Neos Front 9
    CANSparkMax m_BackIndexor = new CANSparkMax(Constants.CAN.kIndexorBack, MotorType.kBrushed); // 775 Back 10
    CANSparkMax m_LowIndexor = new CANSparkMax(Constants.CAN.kIndexorLower, MotorType.kBrushless); //Lower Neo Indexor
    CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.CAN.kIntake, MotorType.kBrushed); // 775 Intake 7
    // CANSparkMax m_shooter = new CANSparkMax(Constants.CAN.kShooter, MotorType.kBrushless);

    public ToggleGeneralMotors() {
        IndexorOff();
        // LowerIndexOff();
        // set it to default stop motor mode unless toggled
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
