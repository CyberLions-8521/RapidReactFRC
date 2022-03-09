// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mastertoggle;
import frc.robot.subsystems.dreadsubsystem.Drivebase;
import frc.robot.subsystems.utilsubsystem.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;
import java.time.Year;

import javax.swing.text.html.HTMLDocument.BlockElement;

import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class LimeLightRange extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Limelight m_kVision;
    private final frc.robot.subsystems.dreadsubsystem.Drivebase m_Drivebase;
    private double m_Horizonal;
    private double m_Vertical;
    private boolean m_hasTarget;
    /**
     * Creates a new ExampleCommand.
     *
     * 
     * 
     * 
     * @param subsystem The subsystem used by this command.
     */

    public LimeLightRange(Drivebase db, Limelight vision){
        m_kVision = vision;
        m_Drivebase = db;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision);
        addRequirements(db);
    }





    /*____     ______   ____                                                                                                                                                                                                                      
    /\  _`\  /\__  _\ /\  _`\                                                                                                                                                                                                                    
    \ \ \L\ \\/_/\ \/ \ \ \/\ \                                                                                                                                                                                                                  
     \ \ ,__/   \ \ \  \ \ \ \ \                                                                                                                                                                                                                 
      \ \ \/     \_\ \__\ \ \_\ \                                                                                                                                                                                                                
       \ \_\     /\_____\\ \____/                                                                                                                                                                                                                
        \/_/     \/_____/ \/___/                                                                                                                                                                                                                 
                                                                                                                                                                                                                                                 
                                                                                                                                                                                                                                                 
     ____                                          __                                    ___       ______            __                                    ___       ____                                           __                           
    /\  _`\                                       /\ \__  __                            /\_ \     /\__  _\          /\ \__                                /\_ \     /\  _`\                   __                   /\ \__  __                    
    \ \ \L\ \ _ __   ___    _____     ___    _ __ \ \ ,_\/\_\     ___     ___       __  \//\ \    \/_/\ \/      ___ \ \ ,_\     __      __    _ __    __  \//\ \    \ \ \/\ \     __   _ __  /\_\   __  __     __  \ \ ,_\/\_\   __  __     __   
     \ \ ,__//\`'__\/ __`\ /\ '__`\  / __`\ /\`'__\\ \ \/\/\ \   / __`\ /' _ `\   /'__`\  \ \ \      \ \ \    /' _ `\\ \ \/   /'__`\  /'_ `\ /\`'__\/'__`\  \ \ \    \ \ \ \ \  /'__`\/\`'__\\/\ \ /\ \/\ \  /'__`\ \ \ \/\/\ \ /\ \/\ \  /'__`\ 
      \ \ \/ \ \ \//\ \L\ \\ \ \L\ \/\ \L\ \\ \ \/  \ \ \_\ \ \ /\ \L\ \/\ \/\ \ /\ \L\.\_ \_\ \_     \_\ \__ /\ \/\ \\ \ \_ /\  __/ /\ \L\ \\ \ \//\ \L\.\_ \_\ \_   \ \ \_\ \/\  __/\ \ \/  \ \ \\ \ \_/ |/\ \L\.\_\ \ \_\ \ \\ \ \_/ |/\  __/ 
       \ \_\  \ \_\\ \____/ \ \ ,__/\ \____/ \ \_\   \ \__\\ \_\\ \____/\ \_\ \_\\ \__/.\_\/\____\    /\_____\\ \_\ \_\\ \__\\ \____\\ \____ \\ \_\\ \__/.\_\/\____\   \ \____/\ \____\\ \_\   \ \_\\ \___/ \ \__/.\_\\ \__\\ \_\\ \___/ \ \____\
        \/_/   \/_/ \/___/   \ \ \/  \/___/   \/_/    \/__/ \/_/ \/___/  \/_/\/_/ \/__/\/_/\/____/    \/_____/ \/_/\/_/ \/__/ \/____/ \/___L\ \\/_/ \/__/\/_/\/____/    \/___/  \/____/ \/_/    \/_/ \/__/   \/__/\/_/ \/__/ \/_/ \/__/   \/____/
                              \ \_\                                                                                                     /\____/                                                                                                  
                               \/_/                                                                                                     \_/__/                                                                                                   
    */
     // PID constants should be tuned per robot
     final double LINEAR_P = 0.1;
     final double LINEAR_I =0;
     final double LINEAR_D = 0.0;
     PIDController forwardController = new PIDController(LINEAR_P, LINEAR_I ,LINEAR_D);
    
   
   
   public void RangeAndAim(){
    m_Vertical = m_kVision.getTy();
    m_Horizonal = m_kVision.getTx();
    m_hasTarget = m_kVision.getHasTarget();
    double forwardSpeed = 0;
    double desiredDistanceMeters = 0;
    // First calculate range
    double distance = m_kVision.getDistanceToHub();
    if (m_hasTarget==true) {
        
    

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        forwardSpeed = -forwardController.calculate(distance, desiredDistanceMeters);

       
    } else {
        // If we have no targets, turn in place
        forwardSpeed = 0;
      
                      }
        m_Drivebase.autoArcade(forwardSpeed, 0);
   }
    
    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
     

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //setDistance();
      RangeAndAim();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}