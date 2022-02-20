package frc.robot.commands.ToggleshooterIndexor;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterIndexSystem.Indexor;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class ToggleIndex extends CommandBase {
    private final Indexor m_indexor;
  
   
    public ToggleIndex(Indexor subsystem) {
      m_indexor = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  