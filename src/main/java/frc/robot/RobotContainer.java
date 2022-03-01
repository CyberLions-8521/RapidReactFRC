package frc.robot;

import java.sql.DriverPropertyInfo;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
//Additional imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TrajectoryConstants;
// Commands
import frc.robot.Constants.XBOX;
import frc.robot.commands.Drive;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleGear;
import frc.robot.commands.ToggleIntakeArm;

// Subsystems
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.pneumatics.CompressorSystem;
import frc.robot.subsystems.pneumatics.SolenoidsSystem;
import edu.wpi.first.wpilibj2.command.Command;

// Autonomous Mode Imports 
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  // public static final SolenoidsSystem m_solenoids = new SolenoidsSystem();
  // public static final CompressorSystem m_compressor = new CompressorSystem();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  
  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.kXBOX);
  public static final Joystick m_aux = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivebase.setDefaultCommand(m_driveSystem);
    configureButtonBindings();
    //Robot
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeArm(m_solenoids));
    // new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_solenoids));
    // new JoystickButton(m_controller, XBOX.RB).whenPressed(new ToggleCompressor(m_compressor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //ayo?
    //DO NOT
      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
            TrajectoryConstants.ksVolts,
              TrajectoryConstants.kvVoltSecondsPerMeter,
              TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
              TrajectoryConstants.kDriveKinematics,
          10);

 // Create config for trajectory
  TrajectoryConfig config =
      new TrajectoryConfig(
        TrajectoryConstants.kMaxSpeedMetersPerSecond,
        TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(TrajectoryConstants.kDriveKinematics)
         // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

 // An example trajectory to follow.  All units in meters.
 Trajectory exampleTrajectory =
 TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)),
     // Pass through these two interior waypoints, making an 's' curve path
     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(3, 0, new Rotation2d(0)),
     // Pass config
     config);

  RamseteCommand ramseteCommand =
      new RamseteCommand(
          exampleTrajectory,
          m_drivebase::getPose,
          new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
            TrajectoryConstants.ksVolts,
            TrajectoryConstants.kvVoltSecondsPerMeter,
            TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
            TrajectoryConstants.kDriveKinematics,
          m_drivebase::getWheelSpeeds,
          new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
          new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
         // RamseteCommand passes volts to the callback
          m_drivebase::tankDriveVolts,
          m_drivebase);

  String trajectoryJSON = "deploy/redTarmac1.wpilib.json"; //this is where outjson file goes
  Trajectory trajectory = new Trajectory();

  //???????????????????? just added, will need it
  @Override
  public void robotInit() {
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

 // Reset odometry to the starting pose of the trajectory.
  m_drivebase.resetOdometry(exampleTrajectory.getInitialPose());

 // Run path following command, then stop at the end.
  return ramseteCommand.andThen(() -> m_drivebase.tankDriveVolts(0, 0)); 


    // An ExampleCommand will run in autonomous
    
    return new SequentialCommandGroup();
    //return new SequentialCommandGroup();
    //return m_autoCommand;
  }
}
