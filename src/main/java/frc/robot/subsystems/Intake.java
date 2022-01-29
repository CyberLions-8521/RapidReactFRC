// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.XBOX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.CAN.kIntake, MotorType.kBrushed);

  DoubleSolenoid m_leftDS = new DoubleSolenoid(5, 4);
  DoubleSolenoid m_rightDS = new DoubleSolenoid(0, 1);

  Compressor m_compressor = new Compressor();
  boolean m_isExtended;
  boolean m_isOn;

  boolean compOn;

  public Intake()
  {
    // Starts off as retracted 
    retractArms();
    stopSucc();

   
  }

  public void setMotor(double speed)
  {
    m_IntakeMotor.set(speed);
  }


  public boolean isExtended()
  {
    return m_isExtended;
  }

  
  public void toggleCompressor(XboxController m_controller)
  {
    // if (m_controller.getAButtonPressed())
    // {
    //   if (m_compressor.enabled() && !compOn)
    //   {
    //     compOn = false;
    //     m_compressor.stop();
    //   }
    //   else if (!compOn && !m_compressor.enabled())
    //   {
    //     compOn = true;
    //     m_compressor.start();
    //   }
    // }
  }

  public boolean isOn()
  {
    return m_isOn;
  }

  public void extendArms()
  {
    m_leftDS.set(Value.kForward);
    m_rightDS.set(Value.kForward);
    m_isExtended = true;
  }

  public void retractArms()
  {
    m_leftDS.set(Value.kReverse);
    m_rightDS.set(Value.kReverse);
    m_isExtended = false;
  }

  public void succ()
  {
    m_isOn = true;
    // Testing with a low speed first
    m_IntakeMotor.set(0.10);
  }

  public void stopSucc()
  {
    m_isOn = false;
    m_IntakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Applied Output", m_IntakeMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Output Current", m_IntakeMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Bus Voltage", m_IntakeMotor.getBusVoltage());
    // SmartDashboard.putNumber("Sticky Faults", m_IntakeMotor.getStickyFaults());
    // This method will be called once per scheduler run

    double intakeSpeed = -RobotContainer.m_aux.getRawAxis(1);
    m_IntakeMotor.set(intakeSpeed);
  }
}
