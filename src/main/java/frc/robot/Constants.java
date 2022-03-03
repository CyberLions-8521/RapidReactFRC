package frc.robot;

import java.lang.Math;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants 
{
    public static class DriveConstants
    {
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

    }

    public static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.87;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.67;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double wheelBase = 0.54;

        


        // "lengthUnit": "Meter",
        // "exportUnit": "Always Meters",
        // "maxVelocity": 2.0,
        // "maxAcceleration": 2.0,
        // "wheelBase": 0.54,
        // "gameName": "Barrel Racing Path",
        // "outputDir": ""

    }

    public static class TrajectoryConstants{
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    //Feedforward/Feedback Gains
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kTrackwidthMeters = 0.69;

    //DifferentialDriveKinematics
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

     //Max Trajectory Velocity/Acceleration   
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
    //Rameste Parameter
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

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
    public static class SubsystemConstants {
        public static final double intakeSpeed = 0.5;
        public static final double IndexSpeed = 0.3; // For both front and back indexors
        public static final double IndexLower = 0.2; // Lower indexor intake (Optional Usage)
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

    public static class EncodersConstant 
    {
        public static final int LeftEncoderPort = 0;
        public static final int RightEncoderPort = 1;
        public static final double DistancePerPulse = 0.25;
        //Inches
        public static final int Circumference = 6;
    }    public static class CAN {
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

        // check later
        public static final int DPAD_X = 5;
        public static final int DPAD_Y = 6;

    }

    public enum DriveMode {
        ARCADE, TANK
    }

}
