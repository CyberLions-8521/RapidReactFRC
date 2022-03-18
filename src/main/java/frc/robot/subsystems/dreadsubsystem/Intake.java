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
import frc.robot.commands.mastertoggle.LowerIndexor;

public class Intake extends SubsystemBase {
  // DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
  // 5, 4);
  // DoubleSolenoid m_rightArmDS = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  // DoubleSolenoid m_transLeftDS = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 8);
  // DoubleSolenoid m_transRightDS = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);


  //==================================================================================================

  CANSparkMax m_intake = new CANSparkMax(CAN.INTAKE, MotorType.kBrushed);

  boolean m_autoStatus;
  boolean m_gearStatus;
  boolean m_intakeStatus;
  boolean m_toggleSystemStatus;
  boolean m_final;

  public Intake() {
    // stop all subsystem toggle here
    autoIntakeSystemOff();
  }

  public void setMotor(double speed) {
    m_intake.set(speed);
  }

  // Both Intake Arm and Intake Motor Status;
  public void autoIntakeSystemOn() {
    m_leftArmDS.set(kReverse);
    m_rightArmDS.set(kReverse);
    m_intake.set(0.85);
    m_autoStatus = true;
  }

  public void autoIntakeSystemOff() {
    m_leftArmDS.set(kForward);
    m_rightArmDS.set(kForward);
    m_intake.set(0.0);
    m_autoStatus = false;
  }

  // ^ MAIN============================================================================================

  // Ignore the rest Bottom (extra)
  // DS Binds

  public void extendArms() {
    m_leftArmDS.set(kReverse);
    m_rightArmDS.set(kReverse);
  }

  public void retractArms() {
    m_leftArmDS.set(kForward);
    m_rightArmDS.set(kForward);
  }

  public void disableArms() {
    m_leftArmDS.set(kOff);
    m_rightArmDS.set(kOff);
  }
  
  // ^ MAIN============================================================================================
  public void autointakeOn() {
    m_intake.set(0.85);
  }

  public boolean getAutoStatus() {
    return m_autoStatus;
  }

  // public int getGearStatus() {
  // if (m_transRightDS.get().equals(kForward)) {
  // return 1;
  // } else {
  // return 2;
  // }
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Intake Status", getIntakeStatus());
    //SmartDashboard.putBoolean("Upper Index Status", getIndexStatus());
    SmartDashboard.putBoolean("Toggle Intake System", getAutoStatus());
  }

}
