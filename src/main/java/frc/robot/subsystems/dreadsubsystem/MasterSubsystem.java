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
  // DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
  // DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  // DoubleSolenoid m_transLeftDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
  // DoubleSolenoid m_transRightDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  
  DoubleSolenoid m_transLeftDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid m_transRightDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  
  




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

  public MasterSubsystem() {
    // stop all subsystem toggle here
    autoIntakeSystemOff();
    indexOff();
    lowerIndexOff();
    // setGearOff();

  }

  public void setMotor(double speed) {
    m_frontIndexor.set(speed);
    m_backIndexor.set(speed);
    m_lowIndexor.set(speed);
    m_intake.set(speed);
  }

  // Both Intake Arm and Intake Motor Status;
  public void autoIntakeSystemOn() {
    //m_leftArmDS.set(kForward);
    //m_rightArmDS.set(kForward);
    m_intake.set(0.7);
    m_autoStatus = true;
  }

  public void autoIntakeSystemOff() {
  //   m_leftArmDS.set(kReverse);
  //   m_rightArmDS.set(kReverse);
    m_intake.set(0.0);
    m_autoStatus = false;
  }

  public void extendArms() {
    m_leftArmDS.set(kForward);
    m_rightArmDS.set(kForward);
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

  public void setGearOn() {
    m_transLeftDS.set(kForward);
    m_transRightDS.set(kForward);
    m_gearStatus = true;
  }

  public void setGearOff() {
    m_transLeftDS.set(kReverse);
    m_transRightDS.set(kReverse);
    m_gearStatus = false;
  }

  public boolean getTransmissionStatus() {
    return m_gearStatus;
  }

  // Indexor

  public void indexOn() {
    m_frontIndexor.set(-0.60);
    m_backIndexor.set(-0.70);

    m_indexorStatus = true;
  }

  public void indexOff() {
    m_frontIndexor.set(0.0);
    m_backIndexor.set(0.0);
    m_indexorStatus = false;
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

  // boolean status check
  public Value getArmStatus() {
    return m_rightArmDS.get();
  }

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
    SmartDashboard.putBoolean("Gear Change", getTransmissionStatus());
    SmartDashboard.putBoolean("Lower Index Status", getLowerIndexStatus());
    SmartDashboard.putBoolean("Toggle Intake System", getAutoStatus());
  }

}
