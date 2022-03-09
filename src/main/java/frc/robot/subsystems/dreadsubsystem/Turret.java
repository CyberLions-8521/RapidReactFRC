package frc.robot.subsystems.dreadsubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.XBOX;

import static frc.robot.Constants.PIDConstants.*;

/*
TODO: Tune PID coefficients
The SmartDashboard should have inputs to change the coefficients
Follow the instructions here https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html#tuning-methods
Use PI, but if necessary, extend to full PID
After done, revert any tuning code and update PIDConstants with the new coefficients
*/
public class Turret extends SubsystemBase {
  boolean m_shooterStatus;
  CANSparkMax m_shooter = new CANSparkMax(Constants.CAN.SHOOTER, MotorType.kBrushed);
  RelativeEncoder m_encoder = m_shooter.getEncoder(Type.kQuadrature, 4096);
  double m_targetSpeed;

  PIDController m_shooterPID = new PIDController(P_SHOOTER, I_SHOOTER, D_SHOOTER);

  public Turret() {
    m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast); // Allows wheels to move when motor is active
    m_encoder.setPositionConversionFactor(0.25);
    m_encoder.setVelocityConversionFactor(0.25);
    stopShooter();

    SmartDashboard.putNumber("P Shooter", P_SHOOTER);
    SmartDashboard.putNumber("I Shooter", I_SHOOTER);
    SmartDashboard.putNumber("D Shooter", D_SHOOTER);
  }

  public void ControllerBind(XboxController controller) {
    if (controller.getRawButton(XBOX.RB))
      setSpeed(500);
    else if (controller.getRawButton(XBOX.RB) == false)
      stopShooter();
  }

  public void setSpeed(double speed) {
    m_targetSpeed = speed;
  }

  public void stopShooter() {
    m_shooter.set(0.0);
    m_shooterStatus = false;
  }

  public boolean getShooterStatus() {
    return m_shooterStatus;
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    double p = SmartDashboard.getNumber("P Shooter", P_SHOOTER);
    double i = SmartDashboard.getNumber("I Shooter", I_SHOOTER);
    double d = SmartDashboard.getNumber("D Shooter", D_SHOOTER);
    m_shooterPID.setP(p);
    m_shooterPID.setI(i);
    m_shooterPID.setD(d);

    // config whatever setpoint and calulcate through the PID
    double output = m_shooterPID.calculate(m_encoder.getVelocity(), m_targetSpeed);
    // put calulcated output from PID into motor output
    m_shooter.set(output);
    // SmartDashboard.putNumber("Output", output);

    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Toggle Turret", getShooterStatus());
  }
}