package frc.robot.commands.subtoggle;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imports 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;

public class LowerIndexor extends CommandBase {
    /** Creates a new Drive. */
    private final ToggleGeneralMotors m_motor;
    private boolean m_returnStatus;

    public LowerIndexor(ToggleGeneralMotors subsystem) {
        m_motor = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (m_motor.getLowerIndexStatus() == false) {
            m_motor.lowerIndexON();

        } else {
            m_motor.IndexOff();
        }
        m_returnStatus = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_returnStatus;
    }
}
