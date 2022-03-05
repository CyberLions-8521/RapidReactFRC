package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XBOX;
// Commands
import frc.robot.commands.Drive;
import frc.robot.commands.mastertoggle.Climb;
import frc.robot.commands.mastertoggle.LowerIndexor;
import frc.robot.commands.mastertoggle.Shoot;
import frc.robot.commands.mastertoggle.dstoggle.ToggleGear;
import frc.robot.subsystems.dreadsubsystem.Climber;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;

public class RobotContainer {

  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  private static final Climber m_Climber = new Climber();
  public static final MasterSubsystem m_masterSubsystem = new MasterSubsystem();
  public static final Turret m_shooter = new Turret();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  private final Shoot m_shoot = new Shoot(m_shooter, m_masterSubsystem);
  private final LowerIndexor m_lowindex = new LowerIndexor(m_masterSubsystem);
  private static final Climb m_climb = new Climb(m_Climber);

  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.XBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    m_drivebase.setDefaultCommand(m_driveSystem);
    m_Climber.setDefaultCommand(m_climb);
    // m_shooter.setDefaultCommand(m_shoot);
    // m_genmotor.setDefaultCommand(m_index); // double check
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_masterSubsystem));
    // new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem()); // what is this referring to?
    new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_shooter, m_masterSubsystem));
    new JoystickButton(m_controller, XBOX.LB).whenPressed(new LowerIndexor(m_masterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new SequentialCommandGroup();
  }
  /*
   * return new DrivetrainCommand(m_drive);
   * m_drivebase.resetEncoders();
   * m_drivebase.zeroHeading();
   * return new SequentialCommandGroup(
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(0.9),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, -48.3),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(1.35),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, 54.8),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(2.9),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, 40.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(1.25),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, -45.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(0.8),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, -90.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(1.7),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, -120.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(2.1),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, 42.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(2.9),
   * new WaitCommand(1),
   * new RotateCommand(m_drivebase, 45.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, m_masterSubsystem,
   * 0.4).withTimeout(2.0),
   * new WaitCommand(1),
   * new MoveForwardNSeconds(m_drivebase, null, 0.4).withTimeout(2.0) // Last
   * command is toggling intakeOFF during
   * autonomous MODE
   * 
   * );
   * 
   * Toggle Motor During Autonomouse Trajectory Mode
   * m_genmotor.ToggleIntakeSystemON(m_solenoids, m_genmotor);
   * m_genmotor.ToggleIntakeSystemOFF(m_solenoids, m_genmotor);
   * 
   * Trajectory Paths (Uncomment to Choose one)
   * TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.STRAIGHT,
   * m_drivebase);
   * return
   * TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.
   * STRAIGHTLINE
   * , m_drivebase);
   * return
   * TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.
   * ROTATIONALMOVEMENT,
   * m_drivebase);
   * return
   * TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.CIRCLE,
   * m_drivebase);
   * return
   * TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.
   * THREEBALLAUTO,
   * m_drivebase);
   * return
   * TrajectoryFollower.getRamseteCommand(Constants.TrajectoryConstants.
   * PLANNERTEST,
   * m_drivebase);
   */

}
}
