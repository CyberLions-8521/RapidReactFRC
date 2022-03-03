package frc.robot.subsystems;

//Additional Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotContainer;
//XBOX Controller Imports
import frc.robot.Constants.XBOX;
import edu.wpi.first.wpilibj.XboxController;
// Rev Robotics Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// Constants
import frc.robot.Constants;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.DriveConstants;
//Drivebase Motor-Speed Configurations
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;

public class Drivebase extends SubsystemBase {

  String driveMode = "Drive Mode";
  // trying a smaller value for the rate limit
  SlewRateLimiter filter = new SlewRateLimiter(0.2);

  // Constants to control joystick input
  double SPEED_REDUCER = 0.5;
  double TURN_REDUCER = 0.5;
 

  private final DifferentialDriveOdometry m_odometry;
  public PIDController MoveFowardPID = new PIDController(Constants.PIDConstants.KpD, Constants.PIDConstants.KiD, Constants.PIDConstants.KlD);

  // Left GearBox
  CANSparkMax m_leftMaster = new CANSparkMax(Constants.CAN.kLeftMaster, MotorType.kBrushed);
  CANSparkMax m_leftMiddleSlave = new CANSparkMax(Constants.CAN.kLeftMiddleSlave, MotorType.kBrushed);
  CANSparkMax m_leftSlave = new CANSparkMax(Constants.CAN.kLeftSlave, MotorType.kBrushed);

  // Right GearBox
  CANSparkMax m_rightMaster = new CANSparkMax(Constants.CAN.kRightMaster, MotorType.kBrushed);
  CANSparkMax m_rightMiddleSlave = new CANSparkMax(Constants.CAN.kRightMiddleSlave, MotorType.kBrushed);
  CANSparkMax m_rightSlave = new CANSparkMax(Constants.CAN.kRightSlave, MotorType.kBrushed);

  // Differential drive class
  DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

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

  // Arcade Drive
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  double speed;
  double turnRate;
  // DifferentialDriveOdometry odometry;
  // Gyro
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Drive Mode
  public static DriveMode m_mode;

  public Drivebase() {

    // Default mode is tank drive
    m_mode = DriveMode.ARCADE;
    speed = 0.0;
    turnRate = 0.0;
    m_gyro.calibrate();
    // m_gyro.reset();

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

  }

  @Override
  public void periodic() {
    // m_odometry.update(m_gyro.getRotation2d(), m_LeftEncoder.getDistance(),
    // m_RightEncoder.getDistance());
    double tHeading = getHeading().getDegrees();
    SmartDashboard.putNumber("Heading", tHeading);
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

  /**
   * Drive straight with the help of the 9-axis IMU (that's hopefully not damaged
   * by now lol)
   * 
   * @param speed - A value between -1 and 1
   */
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

    double output = MoveFowardPID.calculate(-getTurnRate(), 0);
    arcadeDrive(speed, output, false);
  }

  public double getThrottle() {
    return speed;
  }

  public double getTurnRate() {
    return turnRate;
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
        driveMode = "Arcade Drive";
      } else if (m_mode == DriveMode.ARCADE) {
        m_mode = DriveMode.TANK;
        driveMode = "Tank Drive";
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
    SmartDashboard.putString("Arcade Drive", driveMode);
  }

  public void arcadeDrive(XboxController controller) {
    TURN_REDUCER = (controller.getRawAxis(XBOX.RIGHT_TRIGGER) > 0) ? 0.4 : 0.5;
    SPEED_REDUCER = (controller.getRawAxis(XBOX.LEFT_TRIGGER) > 0) ? 0.5 : 0.65;

    speed = controller.getRawAxis(XBOX.LEFT_STICK_Y) * SPEED_REDUCER;
    turnRate = controller.getRawAxis(XBOX.RIGHT_STICK_X) * TURN_REDUCER;

    speed = limitSpeed(speed);
    turnRate = limitSpeed(turnRate);

    arcadeDrive(speed, -turnRate, true);

    SmartDashboard.putNumber("Speed", -speed);
    SmartDashboard.putNumber("Turn Rate", turnRate);
  }

  public void autoArcade(double speed, double turn) {
    m_drive.arcadeDrive(speed, turn, false);
  }

  public double limitSpeed(double speed) {
    if (speed > 1.0)
      speed = 0.4;
    // speed = DriveConstants.MAX_OUTPUT;
    else if (speed < -1.0)
      speed = -0.4;
    // speed = -DriveConstants.MAX_OUTPUT;
    return speed;
  }

  public AHRS getGyro() {
    return m_gyro;
  }

}
