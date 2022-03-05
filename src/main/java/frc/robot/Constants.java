package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static class DriveConstants {
    // Just a coefficient to dampen how fast the robot turns
    public static final double STEER_K = 0.1;
    // Highest the robot can turn autonomously
    public static final double MAX_OUTPUT = 0.5; // originally 0.5
    public static final double AUTO_MAX_OUTPUT = 0.3;
    public static final double SPEED_LIMIT = 0.5;
    public static final double DRIVE_SLOW = 0.4;
    public static final double TURN_SLOW = 0.5;
    // Steering adjust is never zero, so we choose a number where the robot is
    // basically centered on the target
    public static final double STEER_THRESHOLD = 3;
    // Constant for the slew rate limiter;
    // Limits the rate of change of a signal (joystick input) to 0.5 units per
    // second
    public static final double RATE_LIMIT = 0.5;
  }

  public static class TrajectoryConstants {
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    // Feedforward/Feedback Gains
    public static final double S_VOLTS = 0.22;
    public static final double V_VOLTS = 1.98; // unit s/m
    public static final double A_VOLTS = 0.2; // unit s^2/m
    public static final double TRACK_WIDTH = 0.69; // unit m

    // DifferentialDriveKinematics
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH);

    // Max Trajectory Velocity/Acceleration
    public static final double MAX_SPEED = 3; // unit m/s
    public static final double MAX_ACCELERATION = 3; // unit m/s^2

    // Ramsete Parameter
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    // Example value only - as above, this must be tuned for your drive!
    public static final double DRIVE_VEL = 8.5;
  }

  public static class PIDConstants {
    // Drivebase PID
    public static final double P_DRIVE = 0.1;
    public static final double I_DRIVE = 0;
    public static final double D_DRIVE = 0;

    // Shooter PID
    public static final double P_SHOOTER = 0.1;
    public static final double I_SHOOTER = 0.0;
    public static final double D_SHOOTER = 0.0;

    // Elevator {Climber} PID
    public static final double P_CLIMBER = 0.0;
    public static final double I_CLIMBER = 0.0;
    public static final double D_CLIMBER = 0.0;
  }

  public static class SubsystemConstants {
    public static final double INTAKE_SPEED = 0.5;
    public static final double INDEX_SPEED = 0.3; // For both front and back indexors
    public static final double INDEX_LOWER = 0.2; // Lower indexor intake (Optional Usage)
  }

  public static class VisionConstants {
    // Area of the ball in the camera view when the robot stops approaching it
    public static final double BALL_AREA = 0.7;
    public static final double CAMERA_ANGLE = Math.toRadians(35);
    public static final double HEIGHT_OF_CAMERA = 0.4;
    public static final double HEIGHT_OF_TARGET = 2.64;
  }

  public static class ElevatorOutput {
    public static final double POSITION_MIN = 0;
    public static final double POSITION_MAX = 100;
    public static final double ELEVATOR_UP = 12;
    public static final double ELEVATOR_DOWN = 6;
  }

  public static class EncodersConstant {
    public static final int LEFT_ENCODER_PORT = 0;
    public static final int RIGHT_ENCODER_PORT = 1;
    public static final double DISTANCE_PER_PULSE = 0.25;
    public static final int CIRCUMFERENCE = 6; // Inches
  }

  public static class CAN {
    // Left + right Cim Motors Slave Masters
    public static final int LEFT_MASTER = 3;
    public static final int RIGHT_MASTER = 4;
    // Entirely Left Side Gear Box
    public static final int LEFT_SLAVE = 5;
    public static final int LEFT_MIDDLE_SLAVE = 1; // Find new CANSparkMotor and SetValue to 7
    // Entirely Right Side Gear Box
    public static final int RIGHT_SLAVE = 6;
    public static final int RIGHT_MIDDLE_SLAVE = 2; // Find new CANSparkMotor and SetValue to 6

    // Rest of the Subsystems...
    public static final int INTAKE = 7; // Find new CANSparkMotor and SetValue to 5
    public static final int INDEXOR_LOWER = 8; // 775 Motor
    public static final int INDEXOR_FRONT = 9; // This is inversed motors Neo // Make ths 775
    public static final int INDEXOR_BACK = 10; // Default motor SpinRate Neo
    public static final int SHOOTER = 11; // 775 Motors
    public static final int ELEVATOR = 12; // Neo Motors Brushless with PID
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
    // check later
    public static final int DPAD_X = 5;
    public static final int DPAD_Y = 6;
  }

  public enum DriveMode {
    ARCADE, TANK
  }
}
