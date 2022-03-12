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

  public static class PIDConstants {
    // Drive PID
    public static final double P_DRIVE = 0.02;
    public static final double I_DRIVE = 0.016;
    public static final double D_DRIVE = 0.0;

    // Shooter PID
    public static final double P_SHOOTER = 0.1;
    public static final double I_SHOOTER = 0.0;
    public static final double D_SHOOTER = 0.0;

    // Elevator {Climber} PID
    public static final double P_CLIMBER = 0.0;
    public static final double I_CLIMBER = 0.0;
    public static final double D_CLIMBER = 0.0;
  }

  public static class TrajectoryConstants {
    // Feedforward/Feedback Gains
    public static final double FF_MAX_SPEED = 0.87; // unit m/s
    public static final double FF_MAX_ACCELERATION = 0.67; // unit m/s^2

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double WHEEL_BASE = 0.54;
    public static final double KS = 0.771205; // ksVolts
    public static final double KV = 1.329; // kvVoltSecondsPerMeter
    public static final double KA = 0.0933; // kaVoltSecondsSquaredPerMeter
    public static final double KP = 0.681715; // kTrackwidthMeters
    public static final double P_DRIVE = 0.79;
    // Rameste Parameter
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    // Max Trajectory Velocity/Acceleration
    public static final double MAX_VELOCITY = 0.78;
    public static final double MAX_ACCELERATION = 0.58;
    public static final double STARTING_POSE_X = 0;
    public static final double STARTING_POSE_Y = 0;
    public static final boolean IS_GYRO_REVERSED_FOR_PATHWEAVER = true;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        DriveConstants.TRACK_WIDTH_METERS);
    public static final SimpleMotorFeedforward SIMPLE_MOTOR_FEED_FOrWARD = new SimpleMotorFeedforward(
        TrajectoryConstants.KS, TrajectoryConstants.KV, TrajectoryConstants.KA);
    // Create a voltage constraint to ensure we don't accelerate too fast
    public static final DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            TrajectoryConstants.KS,
            TrajectoryConstants.KV,
            TrajectoryConstants.KA),
        TrajectoryConstants.DRIVE_KINEMATICS,
        8);

    // Create config for trajectory
    public static final TrajectoryConfig CONFIG = new TrajectoryConfig(
        TrajectoryConstants.MAX_VELOCITY,
        TrajectoryConstants.MAX_ACCELERATION)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(TrajectoryConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(AUTO_VOLTAGE_CONSTRAINT);

    // An example trajectory to follow. All units in meters.
    public static final Trajectory STRAIGHT = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        CONFIG);

    public static final Trajectory SPLINE = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(3, 3, Rotation2d.fromDegrees(0))),
        CONFIG);

    public static final Trajectory STRAIGHTLINE = TrajectoryConstants.getTrajectory("Straight");
    public static final Trajectory ROTATIONALMOVEMENT = TrajectoryConstants.getTrajectory("RotationalMovement");
    public static final Trajectory CIRCLE = TrajectoryConstants.getTrajectory("Circle");
    public static final Trajectory THREEBALLAUTO = TrajectoryConstants.getTrajectory("threeballAuto");
    public static final Trajectory PLANNERTEST = TrajectoryConstants.getTrajectory("PPlannerTESTSimple");

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

  public static class DriveConstants {
    // Just a coefficient to dampen how fast the robot turns
    public static final double STEER_K = 0.1;
    // Highest the robot can turn autonomously
    public static final double MAX_OUTPUT = 0.7; // originally 0.5
    public static final double AutoMAX_OUTPUT = 0.3;
    public static final double Speedlimit = 0.6;
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
    public static final double TRACK_WIDTH = 1.178496599;
  }

  public static class VisionConstants {
    // Area of the ball in the camera view when the robot stops approaching it
    public static final double BALL_AREA = 0.7;
    public static final double CAMERA_ANGLE = Math.toRadians(30);
    public static final double CAMERA_HEIGHT = 2.788714;
    public static final double TARGET_HEIGHT = 8.67;
  }

  public static class ElevatorOutput {
    public static final double MIN_POSITION = 0;
    public static final double MAX_POSITION = 100;
    public static final double ELEVATOR_UP = 12;
    public static final double ELEVATOR_DOWN = -6;
  }

  public static class EncodersConstant {
    public static final int LEFT_ENCODER_PORT = 1;
    public static final int RIGHT_ENCODER_PORT = 0;
    public static final double DISTANCE_PER_PULSE = 0.167;
    public static final double CIRCUMFERENCE = 0.5; // ft
  }

  public static class CAN {
    // Left + right Cim Motors Slave Masters
    public static final int LEFT_MASTER = 3;
    public static final int RIGHT_MASTER = 4;
    // Entirely Left Side Gear Box
    public static final int LEFT_SLAVE = 5;
    public static final int LEFT_MID_SLAVE = 1;
    // Entirely Right Side Gear Box
    public static final int RIGHT_SLAVE = 6;
    public static final int RIGHT_MID_SLAVE = 2;
    // Rest of the Subsystems...
    public static final int INTAKE = 7;
    public static final int INDEXOR_LOWER = 8;
    public static final int INDEXOR_FRONT = 9;
    public static final int INDEXOR_BACK = 10;
    public static final int SHOOTER = 11;
    public static final int ELEVATOR = 12;
    public static final int SHAFT_ENCODER = 13;
  }

  public static class IO {
    public static final int XBOX = 0;
    public static final int AUX_CTRL = 1;
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
    public static final int LEFT_POV = 270;
    public static final int RIGHT_POV = 90;
    public static final int UP_POV = 0;
    public static final int DOWN_POV = 180;
  }

  public enum DriveMode {
    ARCADE, TANK
  }

}
