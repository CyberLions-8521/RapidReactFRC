package frc.robot.commands.autonomous;

import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.togglesystem.ToggleGeneralMotors;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class TrajectoryFollower {

    boolean m_trajectoryStatus;
    boolean m_pathCompletition;
    // classes

    public static Command getRamseteCommand(Trajectory path, Drivebase m_drive) {
        RamseteCommand auto = new RamseteCommand(
            path, 
            m_drive::getPose, 
            new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA), 
            // AutoConstants.autoMotorFeedforward, 
            new SimpleMotorFeedforward(TrajectoryConstants.KS, TrajectoryConstants.KV, TrajectoryConstants.KA),
            TrajectoryConstants.DRIVE_KINEMATICS, 
            m_drive::getWheelSpeeds, 
            new PIDController(TrajectoryConstants.kPDriveVel, 0, 0), 
            new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
            m_drive::tankDriveVolts, 
            m_drive);
        
        m_drive.resetOdometry(path.getInitialPose());

        return auto.andThen(() -> m_drive.tankDriveVolts(0,0));
    }
}