package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {
    private final Shooter m_shooter;
    private boolean m_isDone;
    private double speed;

    public Shoot(Shooter shooter) {
        m_shooter = shooter;
        m_isDone = false;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
      /*  m_shooter.setSpeed(-500);
        if (m_shooter.getShooterStatus() == false) {
        } else {
            m_shooter.stopShooter();
        }
        m_isDone = true;
    */}

    
    @Override
    public void execute() {
        m_shooter.ControllerBind(RobotContainer.m_controller);


        
       // m_shooter.setSpeed(-500);
        // if (m_shooter.getSpeed() >= 0.95) {
        //     m_shooter.stopShooter();
        //     m_isDone = true;
        // }
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}