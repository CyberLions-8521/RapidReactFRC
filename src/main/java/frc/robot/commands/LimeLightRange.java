// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.KevinLimelight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;
import java.time.Year;

import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class LimeLightRange extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final KevinLimelight m_kVision;
    private double m_Y;
    /**
     * Creates a new ExampleCommand.
     *
     * 
     * 
     * 
     * @param subsystem The subsystem used by this command.
     */

    public LimeLightRange(KevinLimelight vision) {
        m_kVision=vision;
        
    

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision);
    }

  //Parameters(use meter for height)
    public final double angleOfCamera = 30;
    public final double HeightOfCamera = 0.4;
    public final double HeightOfTarget = 0.6;

    // Math Equation
    public double distance;

    public void setDistance() {

    }

   
    
    @Override
    public void initialize() {
        

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //setDistance();
        m_Y=m_kVision.getTy();
        distance = ((HeightOfTarget - HeightOfCamera) / Math.tan(angleOfCamera+m_Y));
        SmartDashboard.putNumber("Distance", getDistance());

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
    public double getDistance(){

        return distance;
    }
    
}