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

public class Turret extends SubsystemBase {
  boolean m_shooterStatus;
  CANSparkMax m_shooter = new CANSparkMax(Constants.CAN.SHOOTER, MotorType.kBrushed);
  RelativeEncoder m_encoder = m_shooter.getEncoder(Type.kQuadrature, 4096);
  double m_targetSpeed;

  double kp = PIDConstants.P_SHOOTER;
  double ki = PIDConstants.I_SHOOTER;
  double kd = PIDConstants.D_SHOOTER;

  public Turret() {
    m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast); // Allows wheels to move when motor is active
    m_encoder.setPositionConversionFactor(0.25);
    m_encoder.setVelocityConversionFactor(0.25);
    stopShooter();

    SmartDashboard.putNumber("P Shooter", kp);
    SmartDashboard.putNumber("I Shooter", ki);
    SmartDashboard.putNumber("D Shooter", kd);
  }

  // PID controller to compute output for motor
  PIDController ShooterPID = new PIDController(PIDConstants.P_SHOOTER, PIDConstants.I_SHOOTER,
      PIDConstants.D_SHOOTER);

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
    double p = SmartDashboard.getNumber("P Shooter", kp);
    double i = SmartDashboard.getNumber("I Shooter", ki);
    double d = SmartDashboard.getNumber("D Shooter", kd);

    if (p != kp) {
      kp = p;
      ShooterPID.setP(p);
    }
    if (i != ki) {
      ki = i;
      ShooterPID.setI(i);
    }
    if (d != kd) {
      kd = d;
      ShooterPID.setD(d);
    }

    // config whatever setpoint and calulcate through the PID
    double output = ShooterPID.calculate(m_encoder.getVelocity(), m_targetSpeed);
    // put calulcated output from PID into motor output
    m_shooter.set(output);
    // SmartDashboard.putNumber("Output", output);

    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Toggle Turret", getShooterStatus());
  }

}