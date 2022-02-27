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
    boolean m_intakeStatus;
    boolean m_indexorStatus;
  

    CANSparkMax m_FrontIndexor = new CANSparkMax(Constants.CAN.kIndexorFront, MotorType.kBrushless); //Neos Front
    CANSparkMax m_BackIndexor = new CANSparkMax(Constants.CAN.kIndexorBack, MotorType.kBrushless); //Neos Back
    CANSparkMax m_LowIndexor = new CANSparkMax(Constants.CAN.kIndexorLower, MotorType.kBrushed); //Lower 775 Indexor
    CANSparkMax m_Intake = new CANSparkMax(Constants.CAN.kIntake, MotorType.kBrushed); //775 Intake System

    public ToggleGeneralMotors() {


        //set it to default stop motor mode unless toggled

    }

    public void setMotor(double speed){
        m_FrontIndexor.set(speed);
        m_BackIndexor.set(speed);
        m_LowIndexor.set(speed);
    }


    public void intakeOn() {
        //setting motors here



        m_intakeStatus = true;
    }

    public void intakeOff() {



        m_intakeStatus = false;
    }

    public void IndexorOn() {
        m_FrontIndexor.set(0.2);
        m_BackIndexor.set(0.2);

        m_indexorStatus = false;

    }

    public void IndexorOff() {

        m_indexorStatus = false;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // SmartDashboard Stuff here

    }

}
