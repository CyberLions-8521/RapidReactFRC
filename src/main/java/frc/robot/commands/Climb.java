package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Climber m_climber;

    public Climb(Climber m_climber) {
        // set m_subsystem equal to whatever you put in Test
        this.m_climber = m_climber;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climber.initializeEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climber.Tarzan(RobotContainer.m_controller);
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