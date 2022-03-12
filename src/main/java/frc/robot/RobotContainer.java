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
import frc.robot.commands.KevinUltraAuto.UltraAuto;
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
import frc.robot.commands.mastertoggle.dstoggle.ToggleGear;
import frc.robot.commands.mastertoggle.toggleReverseIndexSystem;
import frc.robot.subsystems.dreadsubsystem.Climber;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
// import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
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
  // public static final MasterSubsystem m_masterSubsystem = new
  // MasterSubsystem();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
  private final Shoot m_shoot = new Shoot(m_turret, m_vision, m_indexSubsystem);
  private final ToggleIntakeSystem m_toggleintake = new ToggleIntakeSystem(m_intake);
  private final LowerIndexor m_lowindex = new LowerIndexor(m_indexSubsystem);
  private final toggleReverseIndexSystem m_indextoggle = new toggleReverseIndexSystem(m_indexSubsystem);
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
    m_indexSubsystem.setDefaultCommand(m_indextoggle);
    m_vision.setDefaultCommand(m_shoot);

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    //new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_intake));
    new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem(m_intake));
    new JoystickButton(m_controller, XBOX.A).whenPressed(new LowerIndexor(m_indexSubsystem));
    new JoystickButton(m_controller, XBOX.X).whenPressed(new toggleReverseIndexSystem(m_indexSubsystem)); // double
                                                                                                          // check in
                                                                                                          // testing
                                                                                                          // phase -
                                                                                                          // Thien
                                                                                                          // (Inversed
                                                                                                          // Index motor
                                                                                                          // to fix
                                                                                                          // stick ball)

  }

  public Command getAutonomousCommand() {
    System.out.println("In get autonomous Command");

    // CommandBase autoStraight = new AutoStraight(m_drivebase, 0.5,50);

    // return new SequentialCommandGroup(
    // // new AutoStraight(m_drivebase, -0.5).withTimeout(3),
    // autoStraight.withInterrupt(autoStraight::isFinished),
    // new WaitCommand(1),
    // new AutoShoot(m_turret).withTimeout(2)

    // // new ParallelCommandGroup(
    // // new AutoIntakeArm(m_masterSubsystem)
    // // )
    // );
    return new SequentialCommandGroup(
      new ToggleIntakeSystem(m_intake),
      new AutoStraight(m_drivebase, m_intake, -0.40).withTimeout(1.90),
      new WaitCommand(1),
      new AutoRotateCommand(m_drivebase, -90.0),

      new ParallelCommandGroup(
        new AutoShoot(m_turret),
        new SequentialCommandGroup(
          new AutoStraight(m_drivebase,m_intake,-0.40),
          new AutoLowerindex(m_indexSubsystem)
        )
      )

    // new ParallelCommandGroup(
    // new AutoShoot(m_turret),
    // new AutoIntakeArm(m_masterSubsystem)
    // )
    );

    // Kevin Code
    // return new SequentialCommandGroup(
    // new UltraAuto(m_drivebase, m_masterSubsystem, 0.5, 20)

    // );

    // TIMMY: My code here
    // return new AutoStraight(m_drivebase, -0.40).withTimeout(1.90).andThen(
    // new AutoShoot(m_turret).alongWith(new AutoIntakeArm(m_masterSubsystem))
    // );

    // return new SequentialCommandGroup(
    // new AutoStraight(m_drivebase, -0.4).withTimeout(2),
    // );

    // Working (Run each command by line based on time)
    // return new SequentialCommandGroup(
    // new AutoStraight(m_drivebase, -0.4).withTimeout(3),

    // new WaitCommand(1),
    // new AutoStraight(m_drivebase, 0.4),
    // new AutoIntakeArm(m_masterSubsystem)
    // new AutoStraight(m_drivebase, m_masterSubsystem, 0.5),

  }

}
