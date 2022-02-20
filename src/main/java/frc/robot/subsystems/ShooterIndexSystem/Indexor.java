package frc.robot.subsystems.ShooterIndexSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Indexor extends SubsystemBase {
    private boolean m_indexorStatus;


    //Neo motors 
    CANSparkMax m_indexBottom= new CANSparkMax(Constants.CAN.kIndexorBottom, MotorType.kBrushed);
    CANSparkMax m_indexTop = new CANSparkMax(Constants.CAN.kIndexorTop, MotorType.kBrushed);
    public Indexor() {
        // Okay Set stop methods here
        stopIndexor();

    }

    public boolean indexorOn() {
        return m_indexorStatus;
    }


    public void setSpeed(double speed){
        m_indexTop.set(speed);
        m_indexBottom.set(speed);
    }




    public void startIndexor() {

        m_indexTop.set(0.2);
        m_indexBottom.set(0.2);
        m_indexorStatus = true;

    }


    public void stopIndexor() {
        m_indexTop.set(0.0);
        m_indexBottom.set(0.0);
        m_indexorStatus = false;
    }




    @Override
    public void periodic() {
        
        //return SmartDashboard Value here
        SmartDashboard.putBoolean("Indexor Status", m_indexorStatus);

    }

    @Override
    public void simulationPeriodic() {
      
    }
}
