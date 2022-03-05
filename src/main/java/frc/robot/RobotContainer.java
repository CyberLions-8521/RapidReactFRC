package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// Commands
import frc.robot.Constants.XBOX;
import frc.robot.commands.Drive;
import frc.robot.commands.dstoggle.ToggleGear;
import frc.robot.commands.dstoggle.ToggleIntakeSystem;
import frc.robot.commands.subtoggle.Climb;
import frc.robot.commands.subtoggle.LowerIndexor;
import frc.robot.commands.subtoggle.Shoot;
// Climber stuff
import frc.robot.subsystems.Climber;
// Subsystems
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.pneumatics.SolenoidsSystem;
import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;
import frc.robot.subsystems.togglesystem.Turret;

public class RobotContainer {

  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  public static final Turret m_shooter = new Turret();
  public static final Limelight m_limelight = new Limelight();
  public static final SolenoidsSystem m_solenoids = new SolenoidsSystem();
  public static final ToggleGeneralMotors m_genmotor = new ToggleGeneralMotors();
  private static Climber m_Climber = new Climber();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  private final Shoot m_shoot = new Shoot(m_shooter, m_genmotor);
  private final LowerIndexor m_index = new LowerIndexor(m_genmotor);
  private static final Climb m_Climb = new Climb(m_Climber);

  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.XBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    m_drivebase.setDefaultCommand(m_driveSystem);
    m_shooter.setDefaultCommand(m_shoot);
    // m_genmotor.setDefaultCommand(m_index); // double check
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_solenoids));
    new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem(m_solenoids, m_genmotor));
    new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_shooter, m_genmotor));
    new JoystickButton(m_controller, XBOX.LB).whenPressed(new LowerIndexor(m_genmotor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup();
    // return m_autoCommand;
  }
}
