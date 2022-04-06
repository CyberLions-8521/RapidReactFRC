package frc.robot.commands.mastertoggle;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;
import frc.robot.utility.KevinLib;
import frc.robot.utility.KevinLib.SplineInterpolator;
import frc.robot.RobotContainer;
import frc.robot.Constants.XBOX;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;

public class StopShooter extends CommandBase {
  private final Turret m_shooter;
  private boolean m_isDone;
  private boolean m_returnStatus;

  public StopShooter(Turret shooter) {
    m_shooter = shooter;
   
    m_isDone = false;
    addRequirements(shooter);
  }



  @Override
  public void initialize() {


    
   


  }

  @Override
  public void execute() {
    m_shooter.stopShooter();


  }
  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
