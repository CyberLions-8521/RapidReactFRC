package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XBOX;
// Commands
import frc.robot.commands.Drive;
import frc.robot.commands.autonomous.AutoStraight;
import frc.robot.commands.autonomous.AutoRotateCommand;
//import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.Trajectory.AutoTesting;
import frc.robot.commands.autonomous.Trajectory.MoveInFeet;
import frc.robot.commands.autonomous.Trajectory.PIDTurnToAngle;
//import frc.robot.commands.mastertoggle.Climb;
//import frc.robot.commands.mastertoggle.LowerIndexor;
//import frc.robot.commands.mastertoggle.Shoot;
//import frc.robot.commands.mastertoggle.ToggleIntakeSystem;
//import frc.robot.commands.mastertoggle.dstoggle.ToggleGear;
//import frc.robot.commands.mastertoggle.toggleReverseIndexSystem;
//import frc.robot.subsystems.dreadsubsystem.Climber;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
//import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
//import frc.robot.subsystems.dreadsubsystem.Turret;
//import frc.robot.subsystems.utilsubsystem.Limelight;

public class RobotContainer {
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  // private static final Climber m_Climber = new Climber();
  // public static final MasterSubsystem m_masterSubsystem = new MasterSubsystem();
  // public static final Turret m_turret = new Turret();
  // private static final Limelight m_vision = new Limelight();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  // private final Shoot m_shoot = new Shoot(m_turret, m_vision, m_masterSubsystem);
  // private final ToggleIntakeSystem m_toggleintake = new ToggleIntakeSystem(m_masterSubsystem);
  // private final LowerIndexor m_lowindex = new LowerIndexor(m_masterSubsystem);
  // private final toggleReverseIndexSystem m_indextoggle = new toggleReverseIndexSystem(m_masterSubsystem);
  // private static final Climb m_climb = new Climb(m_Climber);

  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.XBOX);
  public static final Joystick m_aux = new Joystick(1);

  public RobotContainer() {
    // Only setDefaultCommand When calling controller in subsystems.
    m_drivebase.setDefaultCommand(m_driveSystem);
    // m_Climber.setDefaultCommand(m_climb);
    // m_turret.setDefaultCommand(m_shoot);
    // m_masterSubsystem.setDefaultCommand(m_shoot);
    // m_vision.setDefaultCommand(m_shoot);

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    // new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_masterSubsystem));
    // new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem(m_masterSubsystem));
    // new JoystickButton(m_controller, XBOX.A).whenPressed(new LowerIndexor(m_masterSubsystem));
    // new JoystickButton(m_controller, XBOX.X).whenPressed(new toggleReverseIndexSystem(m_masterSubsystem)); // double check in testing phase - Thien

  }

  public Command getAutonomousCommand() {
    System.out.println("In get autonomous Command");

    // Working (Run each command by line based on time)
    return new SequentialCommandGroup(
     // new MoveInFeet(m_drivebase, 0.4, 5)
      // new AutoStraight(m_drivebase, 0.3).withTimeout(1.20),
      // new WaitCommand(2),
      new PIDTurnToAngle(m_drivebase, 180).withTimeout(10)
      //new AutoRotateCommand(m_drivebase, 80.0)
      // 360.0)
        // // new AutoTesting(m_drivebase, m_masterSubsystem, m_turret,
        // // -0.4).withTimeout(6),
        // new AutoStraight(m_drivebase, m_masterSubsystem, -0.5).withTimeout(5), // forward Drivebase
        // new WaitCommand(2),
        // new AutoStraight(m_drivebase, m_masterSubsystem, 0.5).withTimeout(5), // reverse drivebase
        // new WaitCommand(2),
        // new AutoRotateCommand(m_drivebase, 90.0), // ClocCounterkwise
        // new WaitCommand(2),
        // new AutoRotateCommand(m_drivebase, -90.0), // Clockwise
        // new WaitCommand(2),
        // new AutoStraight(m_drivebase, m_masterSubsystem, 0.5),
        // new WaitCommand(2),
        // // going to run both auto commands at the same time here
        // new ParallelCommandGroup( // both should run straight and shoot at the same time
        //     new AutoShoot(m_turret, m_masterSubsystem),
        //     new AutoStraight(m_drivebase, m_masterSubsystem, 0.5).withTimeout(3)

        // )
        
        

    );

  }

}
