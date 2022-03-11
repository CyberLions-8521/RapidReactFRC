package frc.robot.commands.autonomous.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class MoveInFeet extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_toggleIntakeSystem;

  double m_InitHeading;
  double m_feet;
  double m_intakeFeet;
  private double m_speed;
  private boolean m_toggle;

  public MoveInFeet(Drivebase db, MasterSubsystem m_Intake, double speedOffset, double distanceinFeet, boolean toggle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_speed = speedOffset;
    m_feet = distanceinFeet;
    m_toggle = toggle;
    
   //m_intakeFeet = activateFeet;
    m_toggleIntakeSystem = m_Intake;

    addRequirements(db);
    addRequirements(m_Intake);
  //  addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    m_db.resetEncoders();
    //m_InitHeading = m_db.getAngle();
  }

  public void BooleanDriveToDistance(){
    if (m_feet > m_db.getAverageEncoderDistance()){
      //m_db.moveForwardStraight(0.2);
      m_db.autoArcade(m_speed, 0);
    } else {
      m_db.autoArcade(0, 0);
      isFinished();
    }
    



  }

  /*
  public void IntakeActivate(){
    if (m_intakeFeet > m_db.getAverageEncoderDistance()){
      m_toggleIntakeSystem.extendArms();
      m_toggleIntakeSystem.autoIntakeSystemOn();
    } else if(m_intakeFeet + 2 > m_db.getAverageEncoderDistance()) {
      m_toggleIntakeSystem.autoIntakeSystemOff();
      m_toggleIntakeSystem.retractArms();
    }*/
    



  

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

  double distance_P = 0.05;
  double distance_I = 0.0;
  double distance_D = 0.0;
  PIDController DistancePID = new PIDController(distance_P, distance_I, distance_D);

  public void PIDMoveinFeet(){
   double output = DistancePID.calculate(m_db.getAverageEncoderDistance(), m_feet);
   if(m_speed + 0.01 < output && m_feet < m_db.getAverageEncoderDistance()){ 
    m_db.autoArcade(output+m_speed, 0);
    } else {
    m_db.autoArcade(0, 0);
    isFinished();
  }
  
  }

  public void ToggleIntake(boolean m_toggle){
    if(m_toggle){
      m_toggleIntakeSystem.autoIntakeSystemOn();
    } else {
      m_toggleIntakeSystem.autoIntakeSystemOff();
    }

  }
  
  @Override
  public void execute() {
    //BooleanDriveToDistanceSelfCorrecting();
    //m_toggleIntakeSystem.autoIntakeSystemOn();
    BooleanDriveToDistance();
    

  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}