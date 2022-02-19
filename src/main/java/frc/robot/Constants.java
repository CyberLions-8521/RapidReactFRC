// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.Math;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static class DriveConstants
    {
        // Just a coefficient to dampen how fast the robot turns
        public static final double STEER_K = 0.5;
        // Highest the robot can turn autonomously
        public static final double MAX_OUTPUT = 1;
        public static final double AutoMAX_OUTPUT = 0.3;
        public static final double Speedlimit=0.5;

        public static final double DRIVE_SLOW = 1;
        public static final double TURN_SLOW = 2;
        // Steering adjust is never zero, so we choose a number where the robot is basically centered on the target
        public static final double STEER_THRESHOLD = 3;
        // Constant for the slew rate limiter;
        // Limits the rate of change of a signal (joystick input) to 0.5 units per second
        public static final double RATE_LIMIT = 0.5;
    }

    public static class VisionConstants
    {   //change to rapid react
        // Area of the ball in the camera view when the robot stops approaching it
        public static final double BALL_AREA = 0.7;
        public static final double CameraAngle = Math.toRadians(35);
        public static final double HeightOfCamera = 0.4;
        public static final double HeightOfTarget = 2.64;
    }

    public static class CAN
    {
        //Left + right Cim Motors Slave Masters
        public static final int kLeftMaster = 3;
        public static final int kRightMaster = 4;
        //Entirely Left Side Gear Box
        public static final int kLeftSlave = 5;
        public static final int kLeftMiddleSlave = 1; //Find new CANSparkMotor and SetValue to 7
        //Entirely Right Side Gear Box
        public static final int kRightSlave = 6;
        public static final int kRightMiddleSlave =2; //Find new CANSparkMotor and SetValue to 6
        //Intake motor775
        //public static final int kIntake = 5; //Find new CANSparkMotor and SetValue to 5
        //public static final int shooter_motor = 0;

    }

    //Thien Please Check what IO means in WPI
    public static class IO 
    {
        
        public static final int kXBOX = 0;
        public static final int kAuxCtrl = 1;
    }

    public static class XBOX
    {
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
    }

    public enum DriveMode
    {
        ARCADE, TANK
    }

}
