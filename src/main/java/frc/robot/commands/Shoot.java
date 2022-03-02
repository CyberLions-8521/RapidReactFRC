package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.togglesystems.Shooter;

public class Shoot extends CommandBase {
    private final Shooter m_shooter;
    private boolean m_isDone;

    public Shoot(Shooter shooter) {
        m_shooter = shooter;
        m_isDone = false;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setSpeed(1);
    }

    @Override
    public void execute() {
        if (m_shooter.getSpeed() >= 0.95) {
            m_shooter.stopShooter();
            m_isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
