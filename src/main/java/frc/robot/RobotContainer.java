package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XBOX;
// Commands
import frc.robot.commands.Drive;
import frc.robot.commands.LimeLightAimAssist;
import frc.robot.commands.IndexerCommand.StopIndexorAndIntake;
import frc.robot.commands.IndexerCommand.ToggleIndexor;
import frc.robot.commands.IndexerCommand.ToggleIntakeSystem;
import frc.robot.commands.IndexerCommand.ToggleLowerIndexor;
import frc.robot.commands.IndexerCommand.toggleReverseIndexSystem;
import frc.robot.commands.autonomous.AutoShoot;
//import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.Trajectory.AutoTesting;
import frc.robot.commands.autonomous.Trajectory.MoveInFeet;
import frc.robot.commands.autonomous.Trajectory.PIDTurnToAngle;
import frc.robot.commands.mastertoggle.Climb;
import frc.robot.commands.mastertoggle.Shoot;
import frc.robot.commands.mastertoggle.StopShooter;
import frc.robot.commands.mastertoggle.dstoggle.RetractArm;
import frc.robot.subsystems.dreadsubsystem.Climber;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.dreadsubsystem.MasterSubsystem;
import frc.robot.subsystems.dreadsubsystem.Turret;
import frc.robot.subsystems.utilsubsystem.Limelight;

public class RobotContainer {
  // Subsystems
  public static Drivebase m_drivebase = new Drivebase();
  private static final Climber m_Climber = new Climber();
  public static final MasterSubsystem m_masterSubsystem = new MasterSubsystem();
  public static final Turret m_turret = new Turret();
  public static final Limelight m_vision = new Limelight();

  // Commands
  private final Drive m_driveSystem = new Drive(m_drivebase);
   private final Shoot m_shoot = new Shoot(m_turret, m_vision, m_masterSubsystem);
  // private final ToggleIntakeSystem m_toggleintake = new ToggleIntakeSystem(m_masterSubsystem);
  // private final LowerIndexor m_lowindex = new LowerIndexor(m_masterSubsystem);
  // private final toggleReverseIndexSystem m_indextoggle = new toggleReverseIndexSystem(m_masterSubsystem);
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
    //m_vision.setDefaultCommand(m_shoot);

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    // new JoystickButton(m_controller, XBOX.LB).whenPressed(new ToggleGear(m_masterSubsystem));
  //   new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeSystem(m_masterSubsystem));
   //  new JoystickButton(m_controller, XBOX.A).whenPressed(new ToggleLowerIndexor(m_masterSubsystem));

    //for more testing
    new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_turret, m_vision, m_masterSubsystem).alongWith(new RetractArm(m_masterSubsystem))).cancelWhenActive(new toggleReverseIndexSystem(m_masterSubsystem)).cancelWhenActive((new ToggleIndexor(m_masterSubsystem)));
    new JoystickButton(m_controller, XBOX.RB).whenReleased(new StopShooter(m_turret));

   // new JoystickButton(m_controller, XBOX.Y).whenPressed(new RetractArm(m_masterSubsystem)).debounce(0.5);
    //for testing
    // new JoystickButton(m_controller, XBOX.RB).whenPressed(new Shoot(m_turret, m_vision, m_masterSubsystem)).cancelWhenActive(new toggleReverseIndexSystem(m_masterSubsystem)).cancelWhenActive((new ToggleIndexor(m_masterSubsystem))).whenInactive(new StopShooter(m_turret));
    new JoystickButton(m_controller, XBOX.X).whenPressed(new toggleReverseIndexSystem(m_masterSubsystem)).cancelWhenActive
    (new Shoot(m_turret, m_vision, m_masterSubsystem)).cancelWhenActive(new ToggleIndexor(m_masterSubsystem));;// double check in testing phase - Thien
    new JoystickButton(m_controller, XBOX.A).whenPressed(new ToggleIntakeSystem(m_masterSubsystem)).cancelWhenActive(new toggleReverseIndexSystem(m_masterSubsystem)).cancelWhenActive
    (new Shoot(m_turret, m_vision, m_masterSubsystem));

    

     new JoystickButton(m_controller, XBOX.X).and(new JoystickButton(m_controller,XBOX.A).whenReleased(new StopIndexorAndIntake(m_masterSubsystem)));



  }

  public Command getAutonomousCommand() {
    System.out.println("In get autonomous Command");

    return new WaitCommand(.2).andThen(new MoveInFeet (m_drivebase, -0.3, 12)).withTimeout(5).alongWith(new WaitCommand(1)).andThen(new AutoShoot(m_turret).alongWith(new ToggleIndexor(m_masterSubsystem)).withTimeout(5).andThen(new WaitCommand(0.5)));

   // return new SequentialCommandGroup( new LimeLightAimAssist(m_vision, m_drivebase));
    // ez
    //return new WaitCommand(.2).andThen(new MoveInFeet (m_drivebase, 0.5, 30)).withTimeout(5).alongWith(new WaitCommand(5)).andThen(new AutoShoot(m_turret).withTimeout(5).andThen(new WaitCommand(0.5)));
    


    // Working (Run each command by line based on time)
    // return new SequentialCommandGroup(
    //  new MoveInFeet(m_drivebase, 0.15, 23).withTimeout(15),
    //  new PIDTurnToAngle(m_drivebase, 180).withTimeout(2.5),
    //  new MoveInFeet(m_drivebase, 0.15, 21).withTimeout(15)


      

        
        

  //  );

  }

}
