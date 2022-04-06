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

public class Shoot extends CommandBase {
  private final Turret m_shooter;
  private final Limelight m_vision;
  private final MasterSubsystem m_index;
  private boolean m_isDone;
  private boolean m_returnStatus;

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

  List<Double> datapointsRPM = Arrays.asList(1.38, 2.56, 4.3);
  List<Double> datapointsDistance = Arrays.asList(1.38, 2.56, 4.3);

  SplineInterpolator shooterModel = SplineInterpolator.createMonotoneCubicSpline(datapointsDistance, datapointsRPM);

  public void RPMAdjust() {
    // double[] measured = {m_vision.getDistanceToHub()};
    // double[] interpolate = KevinLib.interpLinear(datapointsX, datapointsY,
    // measured);
    // double rpm = interpolate[0];
    double rpm = shooterModel.interpolate(m_vision.getDistanceToHub());
    m_shooter.SpaceStateControl(rpm);

  }

  public void SpaceStateTesting(double rpm) {
    m_shooter.SpaceStateControl(rpm);

  }

  public void AutoIndexerTele() {
    double setpoint = m_vision.getDistanceToMotorVelocity();
    // For testing
    /*
     * 
     * /* if(setpoint-50 < m_encoder.getVelocity()){
     * m_index.indexOn();
     * } else {
     * m_index.indexOff();
     * }
     * }
     */

    if (3950 < m_shooter.m_encoder.getVelocity()) {
      m_index.indexOn();
    } else {
      m_index.indexOff();
    }

  }

  @Override
  public void initialize() {

    //m_shooter.m_shooter.set(1);

    
   


  }

  @Override
  public void execute() {
    //m_shooter.SpaceStateControl(40);
    //m_shooter.SpaceStateControl(4000);
    m_shooter.ControllerBindSpeed(RobotContainer.m_controller, 1);
   // AutoIndexerTele();

  }
  @Override
  public boolean isFinished() {
    return m_returnStatus;
  }
}
