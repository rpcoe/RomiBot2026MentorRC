// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.MathBCR;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.DriveConstants.*;


public class DriveTurn extends Command {
  // Parameters from constuctor
  private final RomiDrivetrain drive;
  private final boolean fromDashboard;
  private double percent;
  private double targetAngle;
  private double kP;
  private TurnMode turnMode;
  // Internal variables
  private double targetRel;
  private double gyroStart, leftEncStart, rightEncStart;
  private double leftDistInchRel, rightDistInchRel;
  private double err;
  private double direction;
  private double tolerance = 2.0;   // stop power a little early to compensate for inertia and delay

  // Datalog variables
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogLeftPercent = new DoubleLogEntry(log, "/DriveTurn/LeftPercent");
  private final DoubleLogEntry dLogRightPercent = new DoubleLogEntry(log, "/DriveTurn/RightPercent");
  private final DoubleLogEntry dLogLeftDistInches = new DoubleLogEntry(log, "/DriveTurn/LeftDistInches");
  private final DoubleLogEntry dLogRightDistInches = new DoubleLogEntry(log, "/DriveTurn/RightDistInches");
  private final DoubleLogEntry dLogLeftVelocityIPS = new DoubleLogEntry(log, "/DriveTurn/LeftVelocityIPS");
  private final DoubleLogEntry dLogRightVelocityIPS = new DoubleLogEntry(log, "/DriveTurn/RightVelocityIPS");
  private final DoubleLogEntry dLogGyroAngle = new DoubleLogEntry(log, "/DriveTurn/GyroAngle");
  private final DoubleLogEntry dLogGyroRelAngle = new DoubleLogEntry(log, "/DriveTurn/GyroRelAngle");
  private final DoubleLogEntry dLogEncRelAngle = new DoubleLogEntry(log, "/DriveTurn/EncRelAngle");
  private final DoubleLogEntry dLogError = new DoubleLogEntry(log, "/DriveTurn/Error");
  private final DoubleLogEntry dLogTurnPercent = new DoubleLogEntry(log, "/DriveTurn/TurnPercent");


  public enum TurnMode {
    GYRO_ABSOLUTE,      // Turn to absolute angle, using gyro
    GYRO_RELATIVE,      // Turn relative to current robot facing, using gyro
    ENCODER_RELATIVE    // Turn relative to current robot facing, using wheel encoders
  }

  /**
   * Turns the robot for a desired rotation (in degrees) and rotational speed.
   * Leverages encoders to compare distance.
   *
   * @param percent The percent voltage which the robot will drive (0 to +1.0)
   * @param degrees Degrees to turn, relative to current heading (+=left, -=right)
   * @param turnMode     
   * <p>GYRO_ABSOLUTE = Turn to absolute (field) angle, using gyro
   * <p>GYRO_RELATIVE = Turn relative to current robot facing, using gyro
   * <p>ENCODER_RELATIVE = Turn relative to current robot facing, using wheel encoders
   * @param drive The drive subsystem on which this command will run
   */
  public DriveTurn(double percent, double degrees, TurnMode turnMode, RomiDrivetrain drive) {
    this.targetAngle = MathBCR.normalizeAngle(degrees);
    this.percent = MathUtil.clamp(Math.abs(percent), 0.0, 1.0);
    this.turnMode = turnMode;
    this.drive = drive;
    kP = kP_angleEnc;  //  This kP is different from kP for gyro
    fromDashboard = false;

    primeDataLog();

    addRequirements(drive);
  }

  /**
   * Turns the robot for a desired rotation (in degrees) and rotational speed, based on 
   * input from the dashboard.
   * Leverages encoders to compare distance.
   *
   * @param turnMode     
   * <p>GYRO_ABSOLUTE = Turn to absolute (field) angle, using gyro
   * <p>GYRO_RELATIVE = Turn relative to current robot facing, using gyro
   * <p>ENCODER_RELATIVE = Turn relative to current robot facing, using wheel encoders
   * @param drive The drive subsystem on which this command will run
   */
  public DriveTurn(TurnMode turnMode, RomiDrivetrain drive) {
    this.turnMode = turnMode;
    this.drive = drive;
    fromDashboard = true;
    addRequirements(drive);

    primeDataLog();

    if(SmartDashboard.getNumber("DriveTurn Target", -9999) == -9999) {
      SmartDashboard.putNumber("DriveTurn Target", 90);
    }
    if(SmartDashboard.getNumber("DriveTurn Percent", -9999) == -9999) {
      SmartDashboard.putNumber("DriveTurn Percent", 0.5);
    }
    if(SmartDashboard.getNumber("DriveTurn kPEnc", -9999) == -9999) {
      SmartDashboard.putNumber("DriveTurn kPEnc", kP_angleEnc);
    }
  }

  /**
   * Prime data logging at boot time.
   * (common code for constructor)
   */
  private void primeDataLog() {
    long timeNow = RobotController.getFPGATime();

    dLogLeftPercent.append(-1, timeNow);
    dLogRightPercent.append(-1, timeNow);
    dLogLeftDistInches.append(-1, timeNow);
    dLogRightDistInches.append(-1, timeNow);
    dLogLeftVelocityIPS.append(-1, timeNow);
    dLogRightVelocityIPS.append(-1, timeNow);
    dLogGyroAngle.append(-1, timeNow);
    dLogGyroRelAngle.append(-1, timeNow);
    dLogEncRelAngle.append(-1, timeNow);
    dLogError.append(-1, timeNow);
    dLogTurnPercent.append(-1, timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    drive.stopMotors();
    gyroStart = drive.getGyroRotation();
    leftEncStart = drive.getLeftDistanceInch();
    rightEncStart = drive.getRightDistanceInch();

    DataLogUtil.writeMessageEcho("Start DriveTurn", "gyroStart =", gyroStart); 


    if (fromDashboard) {
      targetRel = SmartDashboard.getNumber("DriveTurn Target", 90);
      targetRel = MathBCR.normalizeAngle( targetRel );
      percent = SmartDashboard.getNumber("DriveTurn Percent", 0.5);
      percent = MathUtil.clamp( Math.abs(percent), 0.0, 1.0);
      kP = SmartDashboard.getNumber("DriveTurn kPEnc", kP_angleEnc);
    } else {
      targetRel = targetAngle;
    }

    if (turnMode == TurnMode.GYRO_ABSOLUTE) {
      targetRel = MathBCR.normalizeAngle(targetRel - gyroStart);
    }

    if (targetRel < 0) {
      direction = -1.0;      //  Positive degrees is Counter Clockwise
      targetRel = Math.abs(targetRel);
    } else {
      direction = 1.0;
    }
    
    DataLogUtil.writeMessage("DriveTurn: Init, rel angle =", targetRel, ", direction =", direction, ", drive percent =", percent, ", kp =", kP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroRel = Math.abs(MathBCR.normalizeAngle(drive.getGyroRotation() - gyroStart));
    double encAngleRel = getAverageTurningAngle();

    if (turnMode == TurnMode.ENCODER_RELATIVE) {
      err =  targetRel - encAngleRel;
    } else {
      err =  targetRel - gyroRel;
    }
    double pct = MathUtil.clamp(err * kP, kS_angle, percent);

    SmartDashboard.putNumber("DriveTurn CurEncAngle", encAngleRel);

    // Log data
    long timeNow = RobotController.getFPGATime();
    dLogLeftPercent.append(drive.getLeftPercent(), timeNow);
    dLogRightPercent.append(drive.getRightPercent(), timeNow);
    dLogLeftDistInches.append(leftDistInchRel, timeNow);
    dLogRightDistInches.append(rightDistInchRel, timeNow);
    dLogLeftVelocityIPS.append(drive.getLeftVelocity(), timeNow);
    dLogRightVelocityIPS.append(drive.getRightVelocity(), timeNow);
    dLogGyroAngle.append(drive.getGyroRotation(), timeNow);
    dLogGyroRelAngle.append(gyroRel, timeNow);
    dLogEncRelAngle.append(encAngleRel, timeNow);
    dLogError.append(err, timeNow);
    dLogTurnPercent.append(pct, timeNow);

    drive.arcadeDrive(0, pct * direction, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopMotors();
    DataLogUtil.writeMessage("DriveTurn: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return err < tolerance;
  }

  /**
   * Absolute value of average angle turned, relative to starting robot angle, based on encoder readings
   * @return angle, in degrees
   */
  private double getAverageTurningAngle() {
    leftDistInchRel = Math.abs(drive.getLeftDistanceInch() - leftEncStart);
    rightDistInchRel = Math.abs(drive.getRightDistanceInch() - rightEncStart);
    return (leftDistInchRel + rightDistInchRel) / 2.0 / kTurnInchPerDegree;
  }

}