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

public class Solenoids extends SubsystemBase {
 
  DoubleSolenoid m_leftArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
  DoubleSolenoid m_rightArmDS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  
  
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



  
  @Override
  public void periodic() {
   

}
}
