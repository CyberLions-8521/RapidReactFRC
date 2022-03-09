package frc.robot.subsystems.dreadsubsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Counter;
// Encoder / PID Only 
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.EncodersConstant;
import frc.robot.Constants.PIDConstants;
// Constants
import frc.robot.Constants.XBOX;
import frc.robot.RobotContainer;

public class Drivebase extends SubsystemBase {
  // Drive Mode
  public static DriveMode m_mode;
  String m_driveMode = "Drive Mode";
  double m_speed;
  double m_turnRate;

  // Constants to control joystick input
  double m_speedReducer = 0.5;
  double m_turnReducer = 0.5;

  // Encoders stuff
  private final Counter m_rightEncoder = new Counter();
  private final Counter m_leftEncoder = new Counter();

  // Gyro
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // autonomous stuff
  private final DifferentialDriveOdometry m_odometry;

  public PIDController m_pid = new PIDController(PIDConstants.P_DRIVE, PIDConstants.I_DRIVE,
      PIDConstants.D_DRIVE);

  // public PIDController m_PID = new PIDController(PIDConstants.KpD, PIDConstants.KiD, PIDConstants.KlD);


  // Left GearBox
  CANSparkMax m_leftMaster = new CANSparkMax(CAN.LEFT_MASTER, MotorType.kBrushed);
  CANSparkMax m_leftMiddleSlave = new CANSparkMax(CAN.LEFT_MID_SLAVE, MotorType.kBrushed);
  CANSparkMax m_leftSlave = new CANSparkMax(CAN.LEFT_SLAVE, MotorType.kBrushed);

  // Right GearBox
  CANSparkMax m_rightMaster = new CANSparkMax(CAN.RIGHT_MASTER, MotorType.kBrushed);
  CANSparkMax m_rightMiddleSlave = new CANSparkMax(CAN.RIGHT_MID_SLAVE, MotorType.kBrushed);
  CANSparkMax m_rightSlave = new CANSparkMax(CAN.RIGHT_SLAVE, MotorType.kBrushed);

  // Differential drive class
  DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public Drivebase() {
    // initializeEncoder
    m_rightEncoder.setDistancePerPulse(EncodersConstant.DISTANCE_PER_PULSE);
    m_leftEncoder.setDistancePerPulse(EncodersConstant.DISTANCE_PER_PULSE);
    // Default mode is tank drive
    m_mode = DriveMode.ARCADE;
    m_speed = 0.0;
    m_turnRate = 0.0;
    m_gyro.calibrate();
    // m_gyro.reset();
    // // Pathlist
    // follow​(CANSparkMax leader, boolean invert) (Slave Followers)
    m_leftSlave.follow(m_leftMaster, false);
    m_rightSlave.follow(m_rightMaster, false);
    m_leftMiddleSlave.follow(m_leftMaster, true);
    m_rightMiddleSlave.follow(m_rightMaster, true);

    // Setting all Motors Idle mode
    m_leftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftMiddleSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightMiddleSlave.setIdleMode(CANSparkMax.IdleMode.kBrake);

    /*
     * Set the minimum time in seconds that it will take for the Spark Max to change
     * from neutral to full throttle.
     */
    // Therefore higher values give slower acceleration.
    m_leftMaster.setOpenLoopRampRate(0.2);
    m_rightMaster.setOpenLoopRampRate(0.2);
    m_leftSlave.setOpenLoopRampRate(0.2);
    m_leftMiddleSlave.setOpenLoopRampRate(0.2);
    m_rightSlave.setOpenLoopRampRate(0.2);
    m_rightMiddleSlave.setOpenLoopRampRate(0.2);

    // reset odometry in drive mode
    // resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_rightEncoder.setDistancePerPulse(EncodersConstant.DISTANCE_PER_PULSE);
    m_leftEncoder.setDistancePerPulse(EncodersConstant.DISTANCE_PER_PULSE);
    m_rightEncoder.setUpSource(EncodersConstant.RIGHT_ENCODER_PORT);
    m_leftEncoder.setUpSource(EncodersConstant.LEFT_ENCODER_PORT);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  public void turnInPlace(double adjust) {
    m_drive.arcadeDrive(0.0, adjust, true);
    // m_drive.tankDrive(adjust, -adjust, true);
  }

  public void moveForward(double speed, double angle) {
    // Increase corrector to make it move to the left more
    // Decrease to make it move more to the right
    // double corrector = 0.93;
    double REDUCTION = 0.05;
    // m_drivze.tankDrive(speed, speed*corrector, false);
    m_drive.arcadeDrive(speed, -angle * REDUCTION, false);
  }

  // PID drive straight for auto fby Kevin
  public void moveForwardStraight(double speed) {


    double output = m_pid.calculate(-getTurnRate(), 0);

    //double output = m_PID.calculate(-getTurnRate(), 0);

    arcadeDrive(speed, output, false);
  }

  //need to be edit
    public void EncoderDirection(){
    if (m_rightMaster.get() > 0){
      m_rightEncoder.setReverseDirection(true);
    } else if (m_rightMaster.get() < 0)
    m_rightEncoder.setReverseDirection(false);

    if (m_leftEncoder.get() > 0){
      m_leftEncoder.setReverseDirection(true);
    } else if (m_rightMaster.get() < 0)
    m_leftEncoder.setReverseDirection(false);
    }
  

  public double getThrottle() {
    return m_speed;
  }

  public double getTurnRate() {
    return m_turnRate;
  }

  public double getGyroTurnRate() {
    return -m_gyro.getRate();
  }

  public void moveForward(double speed) {
    m_drive.arcadeDrive(speed, 0.0, true);
  }

  public CANSparkMax getLeftMotor() {
    return m_leftMaster;
  }

  public CANSparkMax getRightMotor() {
    return m_rightMaster;
  }

  public void driveWithController(XboxController controller) {
    // Use the Y button to switch between ARCADE and TANK
    if (RobotContainer.m_controller.getYButtonPressed()) {
      if (m_mode == DriveMode.TANK) {
        m_mode = DriveMode.ARCADE;
        m_driveMode = "Arcade Drive";
      } else if (m_mode == DriveMode.ARCADE) {
        m_mode = DriveMode.TANK;
        m_driveMode = "Tank Drive";
      }
    }

    switch (m_mode) {
      case TANK:
        // left speed, right speed, squared inputs
        double leftSpeed = controller.getRawAxis(XBOX.LEFT_STICK_Y) * DriveConstants.MAX_OUTPUT;
        double rightSpeed = controller.getRawAxis(XBOX.RIGHT_STICK_Y) * DriveConstants.MAX_OUTPUT;
        SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);
        m_drive.tankDrive(leftSpeed, rightSpeed, false);
        break;
      case ARCADE:
        arcadeDrive(controller);
        break;
    }
    // Display values to smart dashboard
    SmartDashboard.putString("Arcade Drive", m_driveMode);
  }



  public void arcadeDrive(XboxController controller) {
    m_turnReducer = (controller.getRawAxis(XBOX.RIGHT_TRIGGER) > 0) ? 0.4 : 0.5;
    m_speedReducer = (controller.getRawAxis(XBOX.LEFT_TRIGGER) > 0) ? 0.5 : 0.65;

    m_speed = controller.getRawAxis(XBOX.LEFT_STICK_Y) * m_speedReducer;
    m_turnRate = controller.getRawAxis(XBOX.RIGHT_STICK_X) * m_turnReducer;

    m_speed = clampSpeed(m_speed);
    m_turnRate = clampSpeed(m_turnRate);

    arcadeDrive(m_speed, -m_turnRate, true);

    SmartDashboard.putNumber("Speed", -m_speed);
    SmartDashboard.putNumber("Turn Rate", m_turnRate);
  }

  public void autoArcade(double speed, double turn) {
    m_drive.arcadeDrive(speed, turn, false);
  }

  public double clampSpeed(double speed) {
    if (speed > 1.0)
      speed = 0.4;
    // speed = DriveConstants.MAX_OUTPUT;
    else if (speed < -1.0)
      speed = -0.4;
    // speed = -DriveConstants.MAX_OUTPUT;
    return speed;
  }

  // // Reset Encoders
  public void resetEncoders() {
    m_rightEncoder.reset();
    m_leftEncoder.reset();
  }

  // //Returns for encoder
  public Counter getLeftEncoder() {
    return m_leftEncoder;
  }

  public Counter getRightEncoder() {
    return m_rightEncoder;
  }

  // //Get distance from both encoders and avg them for best accuracy
  public double getAverageEncoderDistance() {
    return ((m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0)
        * (Math.PI * EncodersConstant.CIRCUMFERENCE);
  }

  public double getLeftEncoderDistance() {
    return ((m_leftEncoder.getDistance()) * (Math.PI * EncodersConstant.CIRCUMFERENCE));
  }

  public double getRightEncoderDistance() {
    return ((m_rightEncoder.getDistance()) * (Math.PI * EncodersConstant.CIRCUMFERENCE));
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  // reset gyro
  public void zeroHeading() {
    m_gyro.reset();
  }

  // Odometry
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // reset postions
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  // Wheel-Speed stuff
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),
        m_rightEncoder.getRate());
  }

  // Arcade Drive
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
    m_drive.feed();
  }

  @Override
  public void periodic() {
    EncoderDirection();
    SmartDashboard.putNumber("AverageDistance", getAverageEncoderDistance());
    SmartDashboard.putNumber("LeftEncoderDistance", getLeftEncoderDistance());
    SmartDashboard.putNumber("RightEncoderDistance", getRightEncoderDistance());
    // m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
    // m_rightEncoder.getDistance());
    double tHeading = getHeading().getDegrees();
    SmartDashboard.putNumber("Heading", tHeading);
    m_odometry.update(m_gyro.getRotation2d(), m_rightEncoder.getDistance(), m_leftEncoder.getDistance());

  }

}