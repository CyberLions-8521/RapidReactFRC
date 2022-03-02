package frc.robot.subsystems.togglesystems;

//Additional Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

//XBOX Controller Imports
import frc.robot.Constants.XBOX;
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

public class Shooter extends SubsystemBase {
	boolean m_shooterStatus;
	CANSparkMax m_shooter = new CANSparkMax(kShooter, MotorType.kBrushed);
	//SparkMaxPIDController m_shooterPID = m_shooter.getPIDController();
	RelativeEncoder m_encoder = m_shooter.getEncoder(Type.kQuadrature, 4096);

	double m_targetSpeed = 0;

	public Shooter() {
		// m_shooter.restoreFactoryDefaults();

		// m_shooterPID.setP(kShooterP);
		// m_shooterPID.setI(kShooterI);
		// m_shooterPID.setD(kShooterD);
		// m_shooterPID.setIZone(kShooterIz);
		// m_shooterPID.setFF(kShooterFF);
		// m_shooterPID.setOutputRange(-1, 1);

		m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast); // Allows wheels to move when motor is active
		m_encoder.setPositionConversionFactor(0.25);
		m_encoder.setVelocityConversionFactor(0.25);

		stopShooter();
	}

	public void setSpeed(double speed) {
		m_targetSpeed = speed;
	}

	public void setmotorSpeed(double speed) {
		m_shooter.set(speed);
	}

	public void shooterOn() {
		m_shooter.set(-0.7);
		m_shooterStatus = true;
	}

	public void stopShooter() {
		m_shooter.set(0.0);
		m_shooterStatus = false;

		// m_shooter.disable();
	}

	public boolean getShooterStatus() {
		return m_shooterStatus;
	}

	public double getSpeed() {
		return m_encoder.getVelocity();
	}

	@Override
	public void periodic() {
		//m_shooterPID.setReference(m_targetSpeed, CANSparkMax.ControlType.kVelocity);
		SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
		SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
		SmartDashboard.putBoolean("ShooterStatus", getShooterStatus());
	}
}
