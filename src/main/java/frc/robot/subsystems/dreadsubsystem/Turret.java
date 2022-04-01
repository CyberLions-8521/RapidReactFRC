
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
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

import java.util.Calendar;

public class Turret extends SubsystemBase {
  boolean m_shooterStatus;
  CANSparkMax m_shooter = new CANSparkMax(CAN.SHOOTER, MotorType.kBrushed);
  SparkMaxPIDController m_shooterPID = m_shooter.getPIDController();
  public RelativeEncoder m_encoder = m_shooter.getEncoder(Type.kQuadrature, 8192);

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
  
  //future stuff
    // Volts per (radian per second)
    //need to be change per sysid
    private static final double kFlywheelKv = 0.023;

    // Volts per (radian per second squared)
    private static final double kFlywheelKa = 0.001;
  
    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in rotation per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in rotation per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

        //filter  need to be setup
        private final KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            0.020);
  
        
        //PID is like dogwater compared to this
        private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in rotation per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    
        private final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);
  
  //experimental 
  public void SpaceStateControl(double setpoint){
    m_loop.correct(VecBuilder.fill(m_encoder.getVelocity()));
    m_loop.setNextR(setpoint);
    double nextVoltage = m_loop.getU(0);
    m_shooter.setVoltage(nextVoltage);
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

  // For Auto
  public synchronized void setSpeed() {
      m_shooter.set(1);

    }

  

  //Comment out if it doesnt work - Thien 
  public void setSpeedTurret(double speed) {
  m_shooter.set(speed);
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