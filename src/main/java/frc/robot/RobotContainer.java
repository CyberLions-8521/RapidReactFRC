package frc.robot;

import java.util.List;
import frc.robot.Constants;
import frc.robot.Constants.XBOX;

//Additional imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TrajectoryConstants;
// Commands

import frc.robot.commands.Drive;
// import frc.robot.commands.dstoggle.ToggleGear;
// import frc.robot.commands.dstoggle.ToggleIntakeSystem;
// import frc.robot.commands.subtoggle.Shoot;
// import frc.robot.commands.subtoggle.Climb;
// import frc.robot.commands.subtoggle.LowerIndexor;
// // Subsystems
import frc.robot.subsystems.Drivebase;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.pneumatics.SolenoidsSystem;
// import frc.robot.subsystems.togglesystem.Turret;
// import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.MoveForwardNSeconds;
// Autonomous Mode Imports 
import frc.robot.commands.autonomous.TrajectoryFollower;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {

  // private Trajectory[] paths = new Trajectory[] {
  // PATHS.PathWeaver.getTrajectory("FAR_TRENCH"),
  // PATHS.PathWeaver.getTrajectory("FAR_RENDEVOUS"),
  // PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH"),
  // PATHS.PathWeaver.getTrajectory("MIDDLE_RENDEVOUS"),
  // PATHS.PathWeaver.getTrajectory("CLOSE_TRENCH"),
  // PATHS.PathWeaver.getTrajectory("CLOSE_RENDEVOUS"),
  // PATHS.PathWeaver.getTrajectory("BALL_THIEF"), null,
  // PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH_SIDE"), null,
  // PATHS.STRAIGHT_TRAJECTORY_2M,
  // PATHS.S_TRAJECTORY };

  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  // public static final Turret m_shooter = new Turret();
  // public static final Limelight m_limelight = new Limelight();
  // public static final SolenoidsSystem m_solenoids = new SolenoidsSystem();
  // public static final ToggleGeneralMotors m_genmotor = new
  // ToggleGeneralMotors();
  // private static Climber m_Climber = new Climber();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  // private final Shoot m_shoot = new Shoot(m_shooter, m_genmotor);
  // private final LowerIndexor m_index = new LowerIndexor(m_genmotor);
  // private static final Climb m_Climb = new Climb(m_Climber);

  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.kXBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    m_drivebase.setDefaultCommand(m_driveSystem);

    // m_shooter.setDefaultCommand(m_shoot);
    // m_genmotor.setDefaultCommand(m_index); // double check
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    // new JoystickButton(m_controller, XBOX.LB).whenPressed(new
    // ToggleGear(m_solenoids));
    // new JoystickButton(m_controller, XBOX.B).whenPressed(new
    // ToggleIntakeSystem(m_solenoids, m_genmotor));
    // new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_shooter,
    // m_genmotor));
    // new JoystickButton(m_controller, XBOX.LB).whenPressed(new
    // LowerIndexor(m_genmotor));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new DrivetrainCommand(m_drive);
    m_drivebase.resetEncoders();
    m_drivebase.zeroHeading();

    // m_genmotor.ToggleIntakeSystemON(m_solenoids, m_genmotor);
    // m_genmotor.ToggleIntakeSystemOFF(m_solenoids, m_genmotor);


    // Trajectory Paths (Uncomment to Choose one)
    // TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.STRAIGHT,
    // m_drivebase);
    //return TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.STRAIGHTLINE , m_drivebase);
    return TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.ROTATIONALMOVEMENT, m_drivebase);
    // return TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.CIRCLE, m_drivebase);
    // return TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.THREEBALLAUTO, m_drivebase);
    // return TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.PLANNERTEST, m_drivebase);
  }
}
