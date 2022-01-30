// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Import examples
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;


//true Impiorts

import java.sql.DriverPropertyInfo;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Command ONLY IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.CompressorControl;
import frc.robot.commands.ToggleIntakeArms;
import frc.robot.commands.ToggleIntakeMotor;
//Subsystems ONLY IMPORTS
import frc.robot.subsystems.Intake;
import frc.robot.Constants.XBOX;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivebase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivebase m_drivebase = new Drivebase();  //Calling Drivebase.java Subsystem class
  private final Drive m_driveSystem = new Drive(m_drivebase); //Calling Drive.java Command class
  private final Intake m_intake = new Intake();
  // Controller
  public static final XboxController m_controller = new XboxController(Constants.IO.kXBOX);
  public static final Joystick m_aux = new Joystick(1);


	CompressorControl ctest = new CompressorControl(m_intake);	
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivebase.setDefaultCommand(m_driveSystem);
		m_intake.setDefaultCommand(ctest);


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
		new JoystickButton(m_controller, XBOX.B).whenPressed(new ToggleIntakeArms(m_intake));
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
