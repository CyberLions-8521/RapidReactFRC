package frc.robot.commands.autonomous.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class MoveSeconds extends CommandBase {
  Drivebase m_db;
  MasterSubsystem m_toggleIntakeSystem;

 
  double m_feet;
  
  private double m_speed;


  public MoveSeconds(Drivebase db, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_db = db;
    m_speed = speed;
    
   //m_intakeFeet = activateFeet;
    

    addRequirements(db);

  //  addRequirements(intakeSystem);
  }

  @Override
  public void initialize() {
    //m_InitHeading = m_db.getAngle();
  }

 
  
  
  @Override
  public void execute() {
    //BooleanDriveToDistanceSelfCorrecting();
    //m_toggleIntakeSystem.autoIntakeSystemOn();
    m_db.moveForward(m_speed);    

  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}