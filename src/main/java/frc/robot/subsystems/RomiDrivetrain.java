// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;

import static frc.robot.Constants.DriveConstants.*;

public class RomiDrivetrain extends SubsystemBase implements Loggable {
  // DataLog variables
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private final DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> dLogOdometryPose2D = StructLogEntry.create(log, "/RomiDrivetrain/curPose2d", Pose2d.struct);
  private final DoubleLogEntry dLogLeftPercent = new DoubleLogEntry(log, "/RomiDrivetrain/LeftPercent");
  private final DoubleLogEntry dLogRightPercent = new DoubleLogEntry(log, "/RomiDrivetrain/RightPercent");
  private final DoubleLogEntry dLogLeftDistInches = new DoubleLogEntry(log, "/RomiDrivetrain/LeftDistInches");
  private final DoubleLogEntry dLogRightDistInches = new DoubleLogEntry(log, "/RomiDrivetrain/RightDistInches");
  private final DoubleLogEntry dLogLeftVelocityIPS = new DoubleLogEntry(log, "/RomiDrivetrain/LeftVelocityIPS");
  private final DoubleLogEntry dLogRightVelocityIPS = new DoubleLogEntry(log, "/RomiDrivetrain/RightVelocityIPS");
  private final DoubleLogEntry dLogGyroAngle = new DoubleLogEntry(log, "/RomiDrivetrain/GyroAngle");

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(leftDriveMotor);
  private final Spark m_rightMotor = new Spark(rightDriveMotor);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(leftEncoderA, leftEncoderB);
  private final Encoder m_rightEncoder = new Encoder(rightEncoderA, rightEncoderB);
  private double leftEncoderZeroOffset = 0.0;
  private double rightEncoderZeroOffset = 0.0;

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(this::setLeftMotorOutput, this::setRightMotorOutput);
  private final DifferentialDriveOdometry odometry;

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();
  private double gyroZeroOffset = 0.0;

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();


  /**
   * Creates the Romi drivetrain subsystem.
   * @param log FileLog for the robot
   */
  public RomiDrivetrain() {
    logRotationKey = DataLogUtil.allocateLogRotation();     // Get log rotation for this subsystem

    // Initialize the drivetrain hardware
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    zeroEncoders();
    zeroGyroRotation();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    // Sets initial position to (0,0) facing 0 degrees
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroRotation()), 0, 0);  

    // Prime the DataLog to reduce delay when first enabling the robot
    updateLog(true);
  }

  // ************ Motor control methods

  /**
   * Arcade drive method for differential drive platform. The calculated values will be squared to
   * decrease sensitivity at low speeds.
   *
   * @param xaxisSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zaxisRotate The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   * @param squareInputs If true, then decreases the input sensitivity at low speeds, which can give finer
   * joystick control at low speeds but decreases accuracy for autonomous control.  If false, then 
   * motor power is directly proportional to the axis speeds, which improves accuracy for 
   * autonomous control.
   */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate, boolean squareInputs) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
  }

  /**
   * Stops the drive base (turns off both motors)
   */
  public void stopMotors() {
    m_diffDrive.stopMotor();
  }

  /**
   * Sets the power output to the left drive motor.
   * @param percent percent output (+1 = full forward, -1 = full reverse)
   */
  public void setLeftMotorOutput(double percent) {
    percent = MathUtil.clamp(percent, -1.0, 1.0);
    m_leftMotor.set(percent);

    // Call feed() when not using arcade drive or tank drive to run motors to
    // ensure that motor will not cut out due to differential drive safety.
    m_diffDrive.feed();
  }

  /**
   * Sets the power output to the right drive motor.
   * @param percent percent output (+1 = full forward, -1 = full reverse)
   */
  public void setRightMotorOutput(double percent) {
    percent = MathUtil.clamp(percent, -1.0, 1.0);
    m_rightMotor.set(percent);

    // Call feed() when not using arcade drive or tank drive to run motors to
    // ensure that motor will not cut out due to differential drive safety.
    m_diffDrive.feed();
  }

  /**
   * Reads the current power setting for the left motor.
   * @return percent output (+1 = forward, -1 = reverse)
   */
  public double getLeftPercent(){
    return(m_leftMotor.get());
  }

  /**
   * Reads the current power setting for the right motor.
   * @return percent output (+1 = forward, -1 = reverse)
   */
  public double getRightPercent(){
    return(m_rightMotor.get());
  }

  // ************ Encoder methods

  /**
   * Zeros the encoders to read 0 inches.
   */
  public void zeroEncoders() {
    // Do NOT use the encoder .reset() method.  This reset is asynchronous!
    // So, if you .reset() and then immmediately read back the encoder value, then
    // the encoder may still return the value prior to the reset.  This can cause
    // unpredictable robot behavior immediately after the reset.

    // Instead, record the encoder "zero offset", and substract this zero offset
    // when reading the encoder.  Since the robot user java code is single-threaded,
    // getting() the encoder value after zeroing will always return the correct value.
    leftEncoderZeroOffset = m_leftEncoder.getDistance();
    rightEncoderZeroOffset = m_rightEncoder.getDistance();
  }

  /**
   * Gets the distance traveled, as measured by the left encoder.
   * @return distance in inches (+ = forward, - = reverse)
   */
  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance() - leftEncoderZeroOffset;
  }

  /**
   * Gets the distance traveled, as measured by the right encoder.
   * @return distance in inches (+ = forward, - = reverse)
   */
  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance() - rightEncoderZeroOffset;
  }

  /**
   * Gets the distance traveled, as an average of the left and right encoders.
   * @return distance in inches (+ = forward, - = reverse)
   */
  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**  @return Left Velocity in inches/sec */
  public double getLeftVelocity(){
    double lvel = m_leftEncoder.getRate();    
    return lvel;
  }

  /**  @return Right Velocity in inches/sec */
  public double getRightVelocity(){
    double rvel = m_rightEncoder.getRate();   
    return rvel;
  }

  /**
   * @return average velocity, in inches per second
   */
  public double getAverageEncoderVelocity(){
    return (getRightVelocity() + getLeftVelocity()) / 2;
  }
  
  // ************ Gyro methods

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis (vertical axis).
   *
   * @return The current facing angle of the Romi in degrees.  + = left, - = right
   */
  public double getGyroRotation() {    // Redundant with getGyroAngleZ()   -  added for DriveStraightProfile
    return -m_gyro.getAngleZ() - gyroZeroOffset;  //  Field convention is backwards  Positive is left turn
  }

  /** Zeros the gyro to 0 degrees. */
  public void zeroGyroRotation() {
    // Do NOT use the gyro .reset() method.  This reset is asynchronous!
    // So, if you .reset() and then immmediately read back the gyro value, then
    // the gyro may still return the value prior to the reset.  This can cause
    // unpredictable robot behavior immediately after the reset.

    // Instead, record the gyro "zero offset", and substract this zero offset
    // when reading the gyro.  Since the robot user java code is single-threaded,
    // getting() the gyro value after zeroing will always return the correct value.
    gyroZeroOffset = -m_gyro.getAngleZ();
  }

  /**
	 * Zero the gyro position in software so that it correctly returns
   * the current heading.
	 * @param currentHeading current robot angle that the gyro should return
   * after this method adjusts the gyro zero offset
	 */
	public void zeroGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		gyroZeroOffset = -m_gyro.getAngleZ() - currentHeading;
  }

  // ************ Accelerometer methods

  /** Calculate the Pitch of the robot from the X and Z accelerometers and return degrees of tilt or Pitch*/
  public double getPitch() {
    double pitch = -180*(Math.atan2(getAccelX(), getAccelZ())/Math.PI);
    return  pitch; 
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }
  
  // ************ Odometry methods

  /**
   * Get current robot location and facing on the field
   * @return current robot pose, in meters
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }     

  /**
   * Resets the pose estimator to the specified pose.  I.e. defines the robot's
   * position and orientation on the field.
   * This method also resets the gyro, which is required for the pose
   * to properly reset.
   *
   * @param pose The pose to which to set the pose estimator.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   */
  public void resetPose(Pose2d pose) {
    zeroGyroRotation(pose.getRotation().getDegrees());
    zeroEncoders();

    odometry.resetPosition( Rotation2d.fromDegrees(getGyroRotation()),
      Units.inchesToMeters(getLeftDistanceInch()), 
      Units.inchesToMeters(getRightDistanceInch()), 
      pose );
  }

  // ************ Information methods

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about the hopper to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogOdometryPose2D.append(getPose(), timeNow);
      dLogLeftPercent.append(getLeftPercent(), timeNow);
      dLogRightPercent.append(getRightPercent(), timeNow);
      dLogLeftDistInches.append(getLeftDistanceInch(), timeNow);
      dLogRightDistInches.append(getRightDistanceInch(), timeNow);
      dLogLeftVelocityIPS.append(getLeftVelocity(), timeNow);
      dLogRightVelocityIPS.append(getRightVelocity(), timeNow);
      dLogGyroAngle.append(getGyroRotation(), timeNow);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update robot odometry
    double degrees = getGyroRotation();
    double leftMeters = Units.inchesToMeters(getLeftDistanceInch());
    double rightMeters = Units.inchesToMeters(getRightDistanceInch());
    odometry.update(Rotation2d.fromDegrees(degrees), leftMeters, rightMeters);
    Pose2d curPose = getPose();

    if(fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if(DataLogUtil.isMyLogRotation(logRotationKey)) {
      // Update data on SmartDashboard  
      SmartDashboard.putNumber("Pose X", curPose.getX());
      SmartDashboard.putNumber("Pose Y", curPose.getY());
      SmartDashboard.putNumber("Pose Angle", curPose.getRotation().getDegrees());
      SmartDashboard.putNumber("Drive Left Distance", getLeftDistanceInch());
      SmartDashboard.putNumber("Drive Right Distance", getRightDistanceInch());
      SmartDashboard.putNumber("Drive Left Percent", getLeftPercent());
      SmartDashboard.putNumber("Drive Right Percent", getRightPercent());
      SmartDashboard.putNumber("Drive Left Velocity", getLeftVelocity());
      SmartDashboard.putNumber("Drive Right Velocity", getRightVelocity());
      SmartDashboard.putNumber("Drive Gyro Angle", getGyroRotation());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
