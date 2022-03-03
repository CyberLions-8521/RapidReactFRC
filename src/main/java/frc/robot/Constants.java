package frc.robot;

import java.lang.Math;
import java.util.List;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public final class Constants {
    public static class DriveConstants {
        // Just a coefficient to dampen how fast the robot turns
        public static final double STEER_K = 0.1;
        // Highest the robot can turn autonomously
        public static final double MAX_OUTPUT = 0.5; // originally 0.5
        public static final double AutoMAX_OUTPUT = 0.3;
        public static final double Speedlimit = 0.5;
        public static final double DRIVE_SLOW = 0.4;
        public static final double TURN_SLOW = 0.5;
        // Steering adjust is never zero, so we choose a number where the robot is
        // basically centered on the target
        public static final double STEER_THRESHOLD = 3;
        // Constant for the slew rate limiter;
        // Limits the rate of change of a signal (joystick input) to 0.5 units per
        // second
        public static final double RATE_LIMIT = 0.5;

        // unused
        public static final double TRACK_WIDTH_METERS = 0.638;
        public static final double kTrackWidthMeters = 1.178496599;
    }

    public static class TrajectoryConstants {

        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        // Feedforward/Feedback Gains
        public static final double kMaxSpeedMetersPerSecond = 0.87;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.67;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double wheelBase = 0.54;
        // Test one
        public static final double KS = 0.557; // ksVolts
        public static final double KV = 1.329; // kvVoltSecondsPerMeter
        public static final double KA = 0.0933; // kaVoltSecondsSquaredPerMeter
        public static final double KP = 8.5; // kTrackwidthMeters
        public static final double kPDriveVel = 1.79;
        // Rameste Parameter
        public static final double RAMSETE_B = 2.0;
        public static final double RAMSETE_ZETA = 0.7;
        // Max Trajectory Velocity/Acceleration
        public static final double MAX_VELOCITY = 1.78;
        public static final double MAX_ACCELERATION = 1.78;
        public static final double STARTING_POSE_X = 0;
        public static final double STARTING_POSE_Y = 0;
        public static final boolean IS_GYRO_REVERSED_FOR_PATHWEAVER = true;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                DriveConstants.TRACK_WIDTH_METERS);
        public static final SimpleMotorFeedforward SIMPLE_MOTOR_FEED_FOrWARD = new SimpleMotorFeedforward(
                TrajectoryConstants.KS, TrajectoryConstants.KV, TrajectoryConstants.KA);
        // Create a voltage constraint to ensure we don't accelerate too fast
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        TrajectoryConstants.KS,
                        TrajectoryConstants.KV,
                        TrajectoryConstants.KA),
                TrajectoryConstants.DRIVE_KINEMATICS,
                8);

        // Create config for trajectory
        public static final TrajectoryConfig config = new TrajectoryConfig(
                TrajectoryConstants.MAX_VELOCITY,
                TrajectoryConstants.MAX_ACCELERATION)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(TrajectoryConstants.DRIVE_KINEMATICS)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        public static final Trajectory STRAIGHT = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
        

        
        public static Trajectory getTrajectory(String path) {
            try {
                return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath()
                        .resolve("/home/lvuser/deploy/output/" + path + ".wpilib.json"));
            } catch (Exception e) {
                System.out.println("[ERROR] Something bad happened. SMH");
                e.printStackTrace();
                return null;
            }
        }
    }
    public static class PIDConstants {
        // Drivebase PID
        public static final double KpD = 0.1;
        public static final double KiD = 0;
        public static final double KlD = 0;
        // Shooter PID
        public static final double Kp_shooter = 0.1;
        public static final double Ki_shooter = 0.0;
        public static final double Kd_shooter = 0.0;
        // Elevator {Climber} PID
        public static final double Kp_climber = 0.0;
        public static final double Ki_climber = 0.0;
        public static final double Kd_climber = 0.0;
    }
    public static class VisionConstants {
        // Area of the ball in the camera view when the robot stops approaching it
        public static final double BALL_AREA = 0.7;
        public static final double CameraAngle = Math.toRadians(35);
        public static final double HeightOfCamera = 0.4;
        public static final double HeightOfTarget = 2.64;
    }
    public static class ElevatorOutput {
        public static final double PositionMin = 0;
        public static final double PositionMax = 100;
        public static final double ElevatorUp = 12;
        public static final double ElevatorDown = 6;
    }
    public static class EncodersConstant {
        public static final int LeftEncoderPort = 0;
        public static final int RightEncoderPort = 1;
        public static final double DistancePerPulse = 0.25;
        public static final int Circumference = 6; // Inches
    }
    public static class CAN {
        // Left + right Cim Motors Slave Masters
        public static final int kLeftMaster = 3;
        public static final int kRightMaster = 4;
        // Entirely Left Side Gear Box
        public static final int kLeftSlave = 5;
        public static final int kLeftMiddleSlave = 1; // Find new CANSparkMotor and SetValue to 7
        // Entirely Right Side Gear Box
        public static final int kRightSlave = 6;
        public static final int kRightMiddleSlave = 2; // Find new CANSparkMotor and SetValue to 6
        // Rest of the Subsystems...
        public static final int kIntake = 7; // Find new CANSparkMotor and SetValue to 5
        public static final int kIndexorLower = 8; // 775 Motor
        public static final int kIndexorFront = 9; // This is inversed motors Neo // Make ths 775
        public static final int kIndexorBack = 10; // Default motor SpinRate Neo
        public static final int kShooter = 11; // 775 Motors
        public static final int kElevator = 12; // Neo Motors Brushless with PID
        public static final int kShaftEncoder = 13;
    }
    public static class IO {
        public static final int kXBOX = 0;
        public static final int kAuxCtrl = 1;
    }
    public static class XBOX {
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int LOGO_LEFT = 7;
        public static final int LOGO_RIGHT = 8;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;

        // D-Pad
        public static final int leftPOV = 270;
        public static final int rightPOV = 90;
        public static final int upPOV = 0;
        public static final int downPOV = 180;
        // // check later
    }

    public enum DriveMode {
        ARCADE, TANK
    }

}