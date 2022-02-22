// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.XBOX;

public class EncoderTest extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public EncoderTest() {
  
  }
  //private Joystick m_stick;
  private static final int deviceID = 7;
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  public CANSparkMax EncoderMotor = new CANSparkMax(deviceID, MotorType.kBrushless);

  public void EncoderMotor(XboxController controller){
    EncoderMotor.set(controller.getRawAxis(XBOX.LEFT_STICK_Y));

  }
  
  


  @Override
  public void periodic() {
    
    m_encoder = m_motor.getEncoder();
    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
