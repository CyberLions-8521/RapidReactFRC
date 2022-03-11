
package frc.robot.subsystems.dreadsubsystem;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
//Additional Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PIDConstants;
//XBOX Controller Imports
import frc.robot.Constants.XBOX;
import frc.robot.subsystems.utilsubsystem.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

// Rev Robotics Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

// Constants
import static frc.robot.Constants.CAN.*;
import static frc.robot.Constants.PIDConstants.*;

public class Turret extends SubsystemBase {
  boolean m_shooterStatus;
  CANSparkMax m_shooter = new CANSparkMax(CAN.SHOOTER, MotorType.kBrushed);
  SparkMaxPIDController m_shooterPID = m_shooter.getPIDController();
  public RelativeEncoder m_encoder = m_shooter.getEncoder(Type.kQuadrature, 4096);

  public Turret() {
    m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast); // Allows wheels to move when motor is active

    // stopShooter();
  }

  // PID controller to compute output for motor

  public void ControllerBindPID(XboxController controller, double setpoint) {
    if (controller.getRawButton(XBOX.RB)) {
      m_shooterPID.setReference(setpoint, ControlType.kVelocity, 0);
      m_shooterStatus = true;
    } else if (controller.getRawButton(XBOX.RB) == false)
      stopShooter();
  }

  public void ControllerBindSpeed(XboxController controller, double speed) {
    if (controller.getRawButton(XBOX.RB)) {
      m_shooter.set(speed);
      m_shooterStatus = true;
    } else if (controller.getRawButton(XBOX.RB) == false)
      stopShooter();
  }

  public void AutoShoot(Boolean shoot, double setpoint) {
    if (shoot) {
      m_shooterPID.setReference(setpoint, ControlType.kVelocity, 0);
      m_shooterStatus = true;
    } else {
      stopShooter();
      m_shooterStatus = false;
    }

  }

  public void setSpeed() {
    while (true) {
      m_shooter.set(1);

    }

  }

  public void stopShooter() {
    m_shooter.set(0.0);
    m_shooterStatus = false;

    // m_shooter.disable();
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("ShooterStatus", m_shooterStatus);
  }

}