package frc.robot.subsystems.ShooterIndexSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private boolean m_shootStatus;
  


    //Neo motors 
    CANSparkMax m_ShooterMotor = new CANSparkMax(Constants.CAN.shooter_motor, MotorType.kBrushed);
    public Shooter() {
        // Okay Set stop methods here

    }


    public boolean getShooterStatus() {
        return m_shootStatus;
    }






    public void setSpeed(double speed){
        m_ShooterMotor.set(speed);

    }



    public void startShooter() {
        m_ShooterMotor.set(0.2);
        m_shootStatus = true;


    }




    public void stopShooter() {
        m_ShooterMotor.set(0.0);
        m_shootStatus = false;
    }

    //toggle and switch between functions
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter Status", getShooterStatus());
       
    }

    @Override
    public void simulationPeriodic() {
      
    }
}
