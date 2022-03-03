package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


// Subsystems here
import frc.robot.subsystems.pneumatics.SolenoidsSystem;
import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;
import frc.robot.subsystems.togglesystem.Turret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;

// Commands here (May not need to use unless....) testing it
import frc.robot.commands.dstoggle.ToggleGear;
import frc.robot.commands.subtoggle.LowerIndexor;
import frc.robot.commands.subtoggle.Shoot;
import frc.robot.commands.subtoggle.Climb;

// Additional imports
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;




public class sequentialAuto extends CommandBase{
    private final SolenoidsSystem m_intake;
    private final ToggleGeneralMotors m_genmotors; // Controls intake / 3 index Motors
    private final Turret m_turret;
    private final Climber m_climb;



    public sequentialAuto(SolenoidsSystem intake, ToggleGeneralMotors motors, Turret shooter, Climber climb) {
        m_intake = intake;
        m_genmotors = motors;
        m_turret = shooter;
        m_climb = climb;


        addRequirements(intake);
        addRequirements(motors);
        addRequirements(shooter);
        addRequirements(climb);
       


    }

   
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // execute comands here
  }
  

  @Override
  public void end(boolean interrupted) {
    // shooter.stop(); 
    // shooter.stopBackspin();
    // hopper.stopBelt();
    // hopper.stopKickerWheel();
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}

    
