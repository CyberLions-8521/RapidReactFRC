package frc.robot;

import java.sql.DriverPropertyInfo;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.XBOX;
import frc.robot.commands.Drive;
<<<<<<< HEAD
import frc.robot.commands.LimeLightRange;
import frc.robot.commands.LimelightSeek;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.KevinLimelight;
=======
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleGear;
import frc.robot.commands.ToggleIntakeArm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.pneumatics.CompressorSystem;
import frc.robot.subsystems.pneumatics.SolenoidsSystem;
>>>>>>> 0f17830e4931ecd8d7249072c528be37954f442a
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
<<<<<<< HEAD
  // The robot's subsystems and commands are defined here...
  public static Drivebase m_drivebase = new Drivebase();  //Calling Drivebase.java Subsystem class
  private final Drive m_driveSystem = new Drive(m_drivebase); //Calling Drive.java Command class
  public static KevinLimelight m_KevinLimelight = new KevinLimelight();  //Calling Drivebase.java Subsystem class
  private final LimeLightRange m_range = new LimeLightRange(m_KevinLimelight); //Calling Drive.java Command class
 // private final LimelightSeek m_seek = new LimelightSeek(m_drivebase, m_KevinLimelight); //Calling Drive.java Command class
  
=======
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  public static final SolenoidsSystem m_solenoids = new SolenoidsSystem();
  public static final CompressorSystem m_compressor = new CompressorSystem();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
>>>>>>> 0f17830e4931ecd8d7249072c528be37954f442a
  
  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.kXBOX);
  public static final Joystick m_aux = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivebase.setDefaultCommand(m_driveSystem);
    m_KevinLimelight.setDefaultCommand(m_range);
  


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
    new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeArm(m_solenoids));
    new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_solenoids));
    new JoystickButton(m_controller, XBOX.RB).whenPressed(new ToggleCompressor(m_compressor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    

    // An ExampleCommand will run in autonomous
    //return new SequentialCommandGroup();
    return new SequentialCommandGroup();


    //return m_autoCommand;
  }
}
