package frc.robot.commands.mastertoggle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;
import frc.robot.utility.KevinLib;
import frc.robot.RobotContainer;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class Shoot extends CommandBase {
  private final Turret m_shooter;
  private final Limelight m_vision;
  private final MasterSubsystem m_index;
  private boolean m_isDone;

  public Shoot(Turret shooter, Limelight Vision, MasterSubsystem Mastersubsystem) {
    m_shooter = shooter;
    m_vision = Vision;
    m_index = Mastersubsystem;
    m_isDone = false;
    addRequirements(Vision);
    addRequirements(shooter);
    addRequirements(Mastersubsystem);
  }

  private final int targetSpeed = 500;
  private final int maxtargetSpeed = 600;

  public void AutoIndexerTele(){
    double setpoint = m_vision.getDistanceToMotorVelocity();
      //For testing
      /*

        /*  if(setpoint-50 <  m_encoder.getVelocity()){
             m_index.indexOn();
        } else {
        m_index.indexOff();
       }
      }*/
      
      
      if(3970 < m_shooter.m_encoder.getVelocity()){
        m_index.indexOn();
      } else {
        m_index.indexOff();
      }
      
  }
  double[] datapointsX ={1.38, 2.56, 4.3};
  double[] datapointsY = {1.38, 2.56, 4.3};
  public void RPMAdjust(){
    double[] measured = {m_vision.getDistanceToHub()};
    double[] interpolate = KevinLib.interpLinear(datapointsX, datapointsY, measured);
    double rpm = interpolate[0];
  m_shooter.SpaceStateControl(rpm);

  }

  public void SpaceStateTesting(double rpm){
   m_shooter.SpaceStateControl(rpm);

  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_shooter.ControllerBindSpeed(RobotContainer.m_controller, 1);
    AutoIndexerTele();
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
