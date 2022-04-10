package frc.robot.subsystems.dreadsubsystem;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// Pneumatic Dependecies (API)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class MasterSubsystem extends SubsystemBase {

 

// adawakwdmkwadwadkawdjakd


  CANSparkMax m_intake = new CANSparkMax(CAN.INTAKE, MotorType.kBrushed);
  CANSparkMax m_frontIndexor = new CANSparkMax(Constants.CAN.INDEXOR_FRONT, MotorType.kBrushless);
  CANSparkMax m_backIndexor = new CANSparkMax(Constants.CAN.INDEXOR_BACK, MotorType.kBrushed);
  CANSparkMax m_lowIndexor = new CANSparkMax(Constants.CAN.INDEXOR_LOWER, MotorType.kBrushless);

  boolean m_autoStatus;
  boolean m_gearStatus;
  boolean m_lowindexorStatus;
  boolean m_indexorStatus;
  boolean m_intakeStatus;
  boolean m_toggleSystemStatus;
  boolean m_final;
  boolean m_reverseIndexStatus;

  public MasterSubsystem() {
    // stop all subsystem toggle here
    autoIntakeSystemOff();
    indexOff();
    lowerIndexOff();

  }

  public void setMotor(double speed) {
    m_frontIndexor.set(speed);
    m_backIndexor.set(speed);
    m_lowIndexor.set(speed);
    m_intake.set(speed);
  }

  // Both Intake Arm and Intake Motor Status;
  public void autoIntakeSystemOn() {
    m_intake.set(0.85);
    m_autoStatus = true;
  }

  public void autoIntakeSystemOff() {
    m_intake.set(0.0);
    m_autoStatus = false;
  }


  // Double solenoid transmission gear

  // Indexor

  public void indexOn() {
    m_frontIndexor.set(-0.55);
    m_backIndexor.set(-0.70);

    m_indexorStatus = true;
  }

  public void indexOff() {
    m_frontIndexor.set(0.0);
    m_backIndexor.set(0.0);
    m_indexorStatus = false;
  }

  public void reverseIndexOn() {
    m_frontIndexor.set(1);
    m_backIndexor.set(1);
    m_intake.set(-1);
    m_lowIndexor.set(1);
    m_intakeStatus = true;
    m_indexorStatus = true;
    m_lowindexorStatus = true;
    m_reverseIndexStatus = true;
  }
  public void reverseIndexOff() {
    m_frontIndexor.set(0.0);
    m_backIndexor.set(0.0);
    m_lowIndexor.set(0.0);
    m_reverseIndexStatus = false;
    m_intakeStatus = false;
    m_indexorStatus = false;
    m_lowindexorStatus = false;
  }

  public void lowerIndexOn() {
    m_lowIndexor.set(-0.6);
    m_lowindexorStatus = true;
  }

  public void lowerIndexOff() {
    m_lowIndexor.set(0.0);
    m_lowindexorStatus = false;
  }

  // Intake

  public void intakeOn() {
    m_intake.set(0.6);
    m_intakeStatus = true;
  }
  

  public void intakeOff() {
    m_intake.set(0.0);
    m_intakeStatus = false;
  }

  public void autointakeOn() {
    m_intake.set(0.85);
  }

  // boolean status check

  public boolean getAutoStatus() {
    return m_autoStatus;
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

  public boolean getReverseStatus() {
    return m_reverseIndexStatus;
  }


  // public int getGearStatus() {
  //   if (m_transRightDS.get().equals(kForward)) {
  //     return 1;
  //   } else {
  //     return 2;
  //   }
  // }

  
  @Override
  public void periodic() {
    // SmartDashboard.putString("Arm", getArmStatus().toString());
    // SmartDashboard.putBoolean("Intake Status", getIntakeStatus());
    // SmartDashboard.putBoolean("Upper Index Status", getIndexStatus());
    //SmartDashboard.putNumber("Toggle Gear", getGearStatus());
    SmartDashboard.putBoolean("Lower Index Status", getLowerIndexStatus());
    SmartDashboard.putBoolean("Toggle Intake System", getAutoStatus());
  }

}
