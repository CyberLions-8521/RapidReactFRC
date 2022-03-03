package frc.robot.subsystems.togglesystem;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.XBOX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.Constants;


public class Turret extends SubsystemBase {
    boolean m_shooterStatus;
    CANSparkMax m_shooter = new CANSparkMax(Constants.CAN.kShooter, MotorType.kBrushed);
    //SparkMaxPIDController m_shooterPID = m_shooter.getPIDController();
    RelativeEncoder m_encoder = m_shooter.getEncoder(Type.kQuadrature, 4096);
    double totalspeed;


    public Turret() {
        m_shooter.setIdleMode(CANSparkMax.IdleMode.kCoast); // Allows wheels to move when motor is active
        m_encoder.setPositionConversionFactor(0.25);
        m_encoder.setVelocityConversionFactor(0.25);
        stopShooter();
    }
    
    //PID controller to compute output for motor
    PIDController ShooterPID = new PIDController(PIDConstants.Kp_shooter, PIDConstants.Ki_shooter, PIDConstants.Kd_shooter);

    public void ControllerBind(XboxController controller){
        if(controller.getRawButton(XBOX.RB)){
            setSpeed(500);

        } else if(controller.getRawButton(XBOX.RB)==false)
            stopShooter(); 
    }

    public void setSpeed(double setpoint) {
        //config whatever setpoint and calulcate through the PID
        double output = ShooterPID.calculate(m_encoder.getVelocity(), setpoint);
        //put calulcated output from PID into motor output
        m_shooter.set(output); 
        SmartDashboard.putNumber("Output", output);       
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
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
        SmartDashboard.putBoolean("ShooterStatus", getShooterStatus());
    }


}