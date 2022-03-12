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

public class IndexOnly extends SubsystemBase {

  
  




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

  public IndexOnly() {
    // stop all subsystem toggle here
  
    AutoUpperindexOff();
    AutolowerIndexOff();
    
    // setGearOff();

  }

  public void setMotor(double speed) {
    m_frontIndexor.set(speed);
    m_backIndexor.set(speed);
  }

  // Both Intake Arm and Intake Motor Status;
 

  public void AutoUpperindexOn() {
    m_frontIndexor.set(-0.55);
    m_backIndexor.set(-0.70);
    m_indexorStatus = true;
  }

  public void AutoUpperindexOff(){
    m_frontIndexor.set(-0.55);
    m_backIndexor.set(-0.70);

    m_indexorStatus = false;

  }


  public void AutolowerIndexOn() {
    m_lowIndexor.set(-0.6);
    m_lowindexorStatus = true;
  }

  public void AutolowerIndexOff() {
    m_lowIndexor.set(0.0);
    m_lowindexorStatus = false;
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




  
  @Override
  public void periodic() {
   
    // SmartDashboard.putBoolean("Gear Change", getTransmissionStatus());
    // SmartDashboard.putBoolean("Lower Index Status", getLowerIndexStatus());
    // SmartDashboard.putBoolean("Toggle Intake System", getAutoStatus());
  }

}
