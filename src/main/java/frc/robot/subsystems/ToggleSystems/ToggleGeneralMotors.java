package frc.robot.subsystems.ToggleSystems;

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

    CANSparkMax m_FrontIndexor = new CANSparkMax(Constants.CAN.kIndexorFront, MotorType.kBrushless); // Neos Front 9
    CANSparkMax m_BackIndexor = new CANSparkMax(Constants.CAN.kIndexorBack, MotorType.kBrushless); // Neos Back 10
    // CANSparkMax m_LowIndexor = new CANSparkMax(Constants.CAN.kIndexorLower, MotorType.kBrushed); //Lower 775 Indexor
    // CANSparkMax m_shooter = new CANSparkMax(Constants.CAN.kShooter, MotorType.kBrushless);

    public ToggleGeneralMotors() {
        IndexorOff();
        // LowerIndexOff();
        // set it to default stop motor mode unless toggled
    }

    public void setMotor(double speed) {
        m_FrontIndexor.set(speed);
        m_BackIndexor.set(speed);
        // m_LowIndexor.set(speed);
    }

    public boolean getIndexStatus() {
        return m_indexorStatus;
    }

    public void IndexorOn() {
        m_FrontIndexor.set(0.40);
        m_BackIndexor.set(0.40);
        // m_LowIndexor.set(0.40);
        m_indexorStatus = true;
    }

    public void IndexorOff() {
        m_FrontIndexor.set(0.0);
        m_BackIndexor.set(0.0);
        // m_LowIndexor.set(0.0);
        m_indexorStatus = false;
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexor Status", getIndexStatus());

    }

}
