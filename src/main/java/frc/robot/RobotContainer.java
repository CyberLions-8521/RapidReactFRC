package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XBOX;
// Commands
import frc.robot.commands.Drive;
import frc.robot.commands.autonomous.MoveForwardNSecondsTest;
import frc.robot.commands.autonomous.MoveInFeet;
import frc.robot.commands.autonomous.PIDTurnToAngle;
import frc.robot.commands.autonomous.RotateCommand;
import frc.robot.commands.autonomous.ToggleShooter;
import frc.robot.commands.mastertoggle.Climb;
import frc.robot.commands.mastertoggle.LowerIndexor;
import frc.robot.commands.mastertoggle.Shoot;
// import frc.robot.commands.mastertoggle.Shoot;
import frc.robot.commands.mastertoggle.ToggleIntakeSystem;
import frc.robot.commands.mastertoggle.dstoggle.ToggleGear;
import frc.robot.commands.mastertoggle.toggleIndexSystem;
import frc.robot.subsystems.dreadsubsystem.Climber;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
//import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;
import frc.robot.commands.autonomous.AutoIntakeSystem;
import frc.robot.commands.autonomous.AutoMoveForwardNSeconds;
import frc.robot.commands.autonomous.AutoTurretIndex;

public class RobotContainer {

  // SendableChooser<Command> m_chooser = new SendableChooser();
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  private static final Climber m_Climber = new Climber();
  public static final MasterSubsystem m_masterSubsystem = new MasterSubsystem();
  public static final Turret m_turret = new Turret();
  private static final Limelight m_vision = new Limelight();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  private final Shoot m_shoot = new Shoot(m_turret, m_vision, m_masterSubsystem);
  private final ToggleIntakeSystem m_toggleintake = new ToggleIntakeSystem(m_masterSubsystem);
  private final LowerIndexor m_lowindex = new LowerIndexor(m_masterSubsystem);
  private final toggleIndexSystem m_indextoggle = new toggleIndexSystem(m_masterSubsystem);
  private static final Climb m_climb = new Climb(m_Climber);

  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.XBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    // Only setDefaultCommand When calling controller in subsystems.
    m_drivebase.setDefaultCommand(m_driveSystem);
    m_Climber.setDefaultCommand(m_climb);
    m_turret.setDefaultCommand(m_shoot);
    m_masterSubsystem.setDefaultCommand(m_shoot);
    m_vision.setDefaultCommand(m_shoot);

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_masterSubsystem));
    new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem(m_masterSubsystem)); // what is this
                                                                                                     // referring to?
    // new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_turret,
    // m_masterSubsystem));
    new JoystickButton(m_controller, XBOX.A).whenPressed(new LowerIndexor(m_masterSubsystem));
    new JoystickButton(m_controller, XBOX.X).whenPressed(new toggleIndexSystem(m_masterSubsystem)); // This was testing
                                                                                                    // only DO NOT USE
                                                                                                    // (SHooter toggles
                                                                                                    // indexor)

    // elevator done
    // No Shooter isolated
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("In get autonomous Command");

    // Working
    return new SequentialCommandGroup(
        new AutoMoveForwardNSeconds(m_drivebase, m_masterSubsystem, m_turret, -0.4).withTimeout(6),
        new RotateCommand(m_drivebase, 90).withTimeout(5)
    // new AutoIntakeSystem(m_masterSubsystem).withTimeout(10)
    );

  }

  // Trajectory code

  /*
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
