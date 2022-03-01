package frc.robot.subsystems.ToggleSystems;

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

// Constants
import frc.robot.Constants;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.DriveConstants;

//Drivebase Motor-Speed Configurations
import edu.wpi.first.math.filter.SlewRateLimiter;


public class Shooter extends SubsystemBase {
	boolean m_ShooterStatus;
	CANSparkMax m_shooter = new CANSparkMax(Constants.CAN.kShooter, MotorType.kBrushed);
	m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast); //Allows wheels to move when motor is active



	//PId

	//method for stopping motor at cetain RPM


	//Get started on PID Controllers here...
	

	
	public Shooter() {
		//execute command methods here
		stopShooter();
		
		
		
	}

	public void getSpeed(double speed){
		m_shooter.set(speed);
	}

	public boolean getShooterStatus() {
		return m_ShooterStatus;
		
	}

	public void stopShooter() {
		m_shooter.set(0.0)
	}

  @Override
  public void periodic() {

  }
}



