package frc.robot.subsystems.ShooterIndexSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  


    //Neo motors 
    CANSparkMax m_ShooterMotor = new CANSparkMax(Constants.CAN.shooter_motor, MotorType.kBrushed);
    public Shooter() {
        // Okay Set stop methods here

    }


    public void setSpeed(double speed){
        m_ShooterMotor.set(speed);
    }

    //toggle and switch between functions



        









    @Override
    public void periodic() {
        
        //return SmartDashboard Value here
    }

    @Override
    public void simulationPeriodic() {
      
    }
}
