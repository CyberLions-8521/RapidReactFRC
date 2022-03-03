package frc.robot.subsystems;

//Additional Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotContainer;
// Constants
import frc.robot.Constants.XBOX;
import frc.robot.Constants;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.EncodersConstant;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Counter;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// Encoder / PID Only 
// import frc.robot.Constants.EncodersConstants;
// import frc.robot.commands.Drive;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
// LMFAO IMAGINE ADDING MORE IMPORT LMFAOOO PLS HELP ME OML OML I CANT DO THIS NO MORE MAN T_T
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import static frc.robot.Constants.*;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TrajectoryConstants;
import java.util.List;

public class Drivebase extends SubsystemBase {

  String driveMode = "Drive Mode";
  // trying a smaller value for the rate limit
  SlewRateLimiter filter = new SlewRateLimiter(0.2);

  // Constants to control joystick input
  double SPEED_REDUCER = 0.5;
  double TURN_REDUCER = 0.5;

  // autonomous stuff
  private final DifferentialDriveOdometry m_odometry;
  //public PIDController MoveFowardPID = new PIDController(PIDConstants.KpD, PIDConstants.KiD, PIDConstants.KlD);
  public PIDController ramseteController = new PIDController(PIDConstants.KpD, PIDConstants.KiD, PIDConstants.KlD);
  // private final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
  //     TrajectoryConstants.SIMPLE_MOTOR_FEED_FOrWARD, TrajectoryConstants.DRIVE_KINEMATICS, 5);
  // private final TrajectoryConfig config = new TrajectoryConfig(TrajectoryConstants.MAX_VELOCITY,
  //     TrajectoryConstants.MAX_ACCELERATION).setKinematics(TrajectoryConstants.DRIVE_KINEMATICS)
  //         .addConstraint(autoVoltageConstraint);

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

  // Encoders stuff
  private final Counter m_RightEncoder = new Counter();
  private final Counter m_LeftEncoder = new Counter();

  double speed;
  double turnRate;

  // Gyro
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Drive Mode
  public static DriveMode m_mode;

  //public final List<Trajectory> pathList;

  public Drivebase() {

    // Default mode is tank drive
    m_mode = DriveMode.ARCADE;
    speed = 0.0;
    turnRate = 0.0;
    m_gyro.calibrate();
    // m_gyro.reset();

   

    // // Pathlist
    // pathList = List.of(
    //     TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
    //         List.of(new Translation2d(1, 0)), new Pose2d(3, 0, new Rotation2d(0)), config),
    //     TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
    //         List.of(new Translation2d(2, 0)), new Pose2d(4, 0, new Rotation2d(0)), config));
    // resetEncoders();

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

    double output = ramseteController.calculate(-getTurnRate(), 0);
    arcadeDrive(speed, output, false);
  }

  public double getThrottle() {
    return speed;
  }

  public double getTurnRate() {
    return turnRate;
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

  public void initializeEncoder() {
    m_RightEncoder.setDistancePerPulse(EncodersConstant.DistancePerPulse);
    m_LeftEncoder.setDistancePerPulse(EncodersConstant.DistancePerPulse);
    m_RightEncoder.setUpSource(EncodersConstant.RightEncoderPort);
    m_LeftEncoder.setUpSource(EncodersConstant.LeftEncoderPort);
  }

  // private final Encoder m_LeftEncoder = new Encoder(
  // EncodersConstants.m_LeftSlaveEncoderPorts[0],
  // EncodersConstants.m_LeftSlaveEncoderPorts[1],
  // EncodersConstants.m_LeftSlaveEncoderReversed);

  // // Reset Encoders
  public void resetEncoders() {
    m_RightEncoder.reset();
    m_LeftEncoder.reset();
  }

  // //Returns for encoder
  public Counter getLeftEncoder() {
    return m_LeftEncoder;
  }

  public Counter getRightEncoder() {
    return m_RightEncoder;
  }

  // //Get distance from both encoders and avg them for best accuracy
  public double getAverageEncoderDistance() {
    return ((m_LeftEncoder.getDistance() + m_RightEncoder.getDistance()) / 2.0)
        * (Math.PI * EncodersConstant.Circumference);
  }

  public double getLeftEncoderDistance() {
    return ((m_LeftEncoder.getDistance()) * (Math.PI * EncodersConstant.Circumference));
  }

  public double getRightEncoderDistance() {
    return ((m_RightEncoder.getDistance()) * (Math.PI * EncodersConstant.Circumference));
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
    return new DifferentialDriveWheelSpeeds(m_LeftEncoder.getRate(),
        m_RightEncoder.getRate());
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

  // PATHWEABNJDEOWIJDJIUONKL JWDMSNDSK DKOSDM LS

  // /**
  //   * Returns a {@code RamseteCommand} object. Used to follow the specified
  //   * {@link Trajectory}.
  //   *
  //   * @param path the {@code Trajectory} to follow
  //   * @return a {@link RamseteCommand} object
  //   */
  //   public RamseteCommand ramseteCommand(Trajectory path) {
  //     return new RamseteCommand(path, m_odometry::getPoseMeters, new RamseteController(),
  //     Constants.PIDConstants.Somethinghere, Constants.PIDConstants.something here,
  //       this::getWheelSpeeds, MoveFowardPID, MoveFowardPID, this::voltDrive, this);
  //   }

      /**
    * Returns a {@code RamseteCommand} object. Used to follow the specified
    * {@link Trajectory}.
    *
    * @param path the {@code Trajectory} to follow
    * @return a {@link RamseteCommand} object
    */
  // public RamseteCommand ramseteCommand(Trajectory path) {
  //   return new RamseteCommand(path, m_odometry::getPoseMeters, new RamseteController(),
  //     TrajectoryConstants.SIMPLE_MOTOR_FEED_FOrWARD, TrajectoryConstants.DRIVE_KINEMATICS,
  //     this::getWheelSpeeds, ramseteController, ramseteController, this::tankDriveVolts, this);
  // }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AverageDistance", getAverageEncoderDistance());
    SmartDashboard.putNumber("LeftEncoderDistance", getLeftEncoderDistance());
    SmartDashboard.putNumber("RightEncoderDistance", getRightEncoderDistance());
    // m_odometry.update(m_gyro.getRotation2d(), m_LeftEncoder.getDistance(),
    // m_RightEncoder.getDistance());
    double tHeading = getHeading().getDegrees();
    SmartDashboard.putNumber("Heading", tHeading);
    m_odometry.update(m_gyro.getRotation2d(), m_RightEncoder.getDistance(), m_LeftEncoder.getDistance());

  }

}
