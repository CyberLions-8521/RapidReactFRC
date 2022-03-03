package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.lang.Object;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
public class auto extends CommandBase {

    public auto() {
        // public static final double kTrackwidthMeters = 0.69;
        //public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(null);
    }

}
