package frc.robot.commands.autonomous;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class MoveInFeet extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_toggleIntakeSystem;

  double m_InitHeading;
  double m_feet;
  private double m_speed;

  public MoveInFeet(Drivebase db, double speedOffset, double feet) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_speed = speedOffset;
    m_feet = feet;
    //m_toggleIntakeSystem = intakeSystem;

    addRequirements(db);
  //  addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    m_db.getGyro().reset();
    m_db.resetEncoders();
    //m_InitHeading = m_db.getAngle();
  }

  public void BooleanDriveToDistance(){
    if (m_feet > m_db.getAverageEncoderDistance()){
      m_db.autoArcade(m_speed, 0);
    } else {
      m_db.autoArcade(0, 0);
      isFinished();
    }

  }

  
  public void BooleanDriveToDistanceSelfCorrecting(){
    double speed = 0;
    if (m_feet > m_db.getAverageEncoderDistance()){
      speed = m_speed;
    } else if (m_feet < m_db.getAverageEncoderDistance()){
      speed = -0.1;
      if (m_feet > m_db.getAverageEncoderDistance())
      speed = 0;
      isFinished();
    } 
    m_db.autoArcade(speed, 0);
  }


  

  public void PIDMoveinFeet(){
    double distance_P = 0.05;
    double distance_I = 0.0;
    double distance_D = 0.0;
    PIDController DistancePID = new PIDController(distance_P, distance_I, distance_D);
     
   double output = DistancePID.calculate(m_db.getAverageEncoderDistance(), m_feet);
   if(m_speed + 0.01 < output && m_feet < m_db.getAverageEncoderDistance()){ 
    m_db.autoArcade(output+m_speed, 0);
    } else {
    m_db.autoArcade(0, 0);
    isFinished();
  }
  
  }
  
  @Override
  public void execute() {
   // BooleanDriveToDistanceSelfCorrecting();
    //m_toggleIntakeSystem.autoIntakeSystemOn();
    

  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
