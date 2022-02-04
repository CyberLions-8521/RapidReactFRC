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
    
    /**
     * Creates a new ExampleCommand.
     *
     * 
     * 
     * 
     * @param subsystem The subsystem used by this command.
     */

    public LimeLightRange(KevinLimelight vision) {
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry y;
        double m_Y;

        m_kVision = vision;
        y=m_kVision.getTy();
    

        //y = m_kVision.getY();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision);
    }

    // public double getTy()
    // {
    // return ty.getDouble(0.0);
    // }

    // public void setTx(NetworkTableEntry a){
    // this.a = a;
    // }

    // pls = m_Yvalue.getDouble();

    // private double NetworkTableEntry.getDouble(String string) {
    // return 0;}

    public final double angleOfCamera = 15;
    // Meters change
    public final double HeightOfCamera = 0.394;
    public final double HeightOfTarget = 2.5;

    // Math Equation
    public double distance;

    public double setDistance() {
        return distance = ((HeightOfTarget - HeightOfCamera) / Math.tan(angleOfCamera +m_kVision.getY()
        ));
    }
    // How to get the data

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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

    public double getDistance() {
        return distance;
    }
}