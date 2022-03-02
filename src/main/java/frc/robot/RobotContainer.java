package frc.robot;

import java.sql.DriverPropertyInfo;

//Additional imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Commands
import frc.robot.Constants.XBOX;
import frc.robot.commands.Drive;
import frc.robot.commands.Shoot;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleGear;
import frc.robot.commands.ToggleIntakeArm;
import frc.robot.commands.ToggleIndex;
import frc.robot.commands.ToggleIntakeMotors;

// Subsystems
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.pneumatics.CompressorSystem;
import frc.robot.subsystems.pneumatics.SolenoidsIntakeSystem;
import frc.robot.subsystems.togglesystems.Shooter;
import frc.robot.subsystems.togglesystems.ToggleGeneralMotors;
import edu.wpi.first.wpilibj2.command.Command;

// Autonomous Mode Imports 
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();




  public static final ToggleGeneralMotors m_index = new ToggleGeneralMotors();
  public static final Shooter m_shooter = new Shooter();

  // public static final SolenoidsSystem m_solenoids = new SolenoidsSystem();
  // public static final CompressorSystem m_compressor = new CompressorSystem();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  private final Shoot m_shoot = new Shoot(m_shooter);
  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.kXBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    // Configure the button bindings
  //  m_drivebase.setDefaultCommand(m_driveSystem);
      m_shooter.setDefaultCommand(m_shoot);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
   // new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_shooter));
    // new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeArm(m_solenoids));
    // new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_solenoids));
    //new JoystickButton(m_controller, XBOX.RB).whenPressed(new ToggleCompressor(m_compressor));
    new JoystickButton(m_controller, XBOX.X).whenPressed(new ToggleIndex(m_index));
    new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeMotors(m_index));
    new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup();
    //return m_autoCommand;
  }
}
