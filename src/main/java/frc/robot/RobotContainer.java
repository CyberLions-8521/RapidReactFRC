package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XBOX;
// Commands
import frc.robot.commands.Drive;
import frc.robot.commands.autonomous.AutoStraight;
import frc.robot.commands.autonomous.AutoIntakeArm;
import frc.robot.commands.autonomous.AutoLowerindex;
import frc.robot.commands.autonomous.AutoRotateCommand;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.Trajectory.AutoTesting;
import frc.robot.commands.autonomous.Trajectory.MoveInFeet;
import frc.robot.commands.autonomous.Trajectory.PIDTurnToAngle;
import frc.robot.commands.mastertoggle.Climb;
import frc.robot.commands.mastertoggle.LowerIndexor;
import frc.robot.commands.mastertoggle.Shoot;
import frc.robot.commands.mastertoggle.ToggleIntakeSystem;
import frc.robot.subsystems.dreadsubsystem.Climber;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.IndexSubsystem;
import frc.robot.subsystems.dreadsubsystem.Intake;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;

public class RobotContainer {
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  private static final Climber m_Climber = new Climber();
  public static final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  public static final Intake m_intake = new Intake();
  public static final Turret m_turret = new Turret();
  private static final Limelight m_vision = new Limelight();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  private final Shoot m_shoot = new Shoot(m_turret, m_vision, m_indexSubsystem);
  private final ToggleIntakeSystem m_toggleintake = new ToggleIntakeSystem(m_intake);
  private final LowerIndexor m_lowindex = new LowerIndexor(m_indexSubsystem);
  private static final Climb m_climb = new Climb(m_Climber);

  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.XBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    // Only setDefaultCommand When calling controller in subsystems.
    m_drivebase.setDefaultCommand(m_driveSystem);
    m_Climber.setDefaultCommand(m_climb);
    m_turret.setDefaultCommand(m_shoot);
    m_indexSubsystem.setDefaultCommand(m_shoot);
    m_vision.setDefaultCommand(m_shoot);

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    /*
    * Turret Shooter is binded via Turret class
    * Toggle Drive_mode is toggled via Drivebase class
    * 
    */
    new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem(m_intake));
    new JoystickButton(m_controller, XBOX.A).whenPressed(new LowerIndexor(m_indexSubsystem));


  }

  public Command getAutonomousCommand() {
    System.out.println("In get autonomous Command");
    return new SequentialCommandGroup(
        // new UltraAuto(m_drivebase, m_indexSubsystem, 0.5, 20)
        new AutoStraight(m_drivebase, -0.3).withTimeout(1.93)

    );


  }

}
