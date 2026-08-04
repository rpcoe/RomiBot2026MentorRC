// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.utilities.DataLogUtil;

public class DriveTime extends Command {

  // Parameters from constructor
  private final double m_duration;    // How long to drive, in milliseconds
  private final double m_percent;     // Desired speed for driving
  private final RomiDrivetrain m_drive;  // Drivetrain subsystem

  // Time when the command starts running
  private long m_startTime;       // Timestamp in milliseconds

  /**
   * Drives the robot for a desired percent speed and time, then stops moving.
   *
   * @param percent The percent speed which the robot will drive, [-1.0..1.0]. Forward is positive.
   * @param time How long to drive, in seconds
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveTime(double percent, double time, RomiDrivetrain romiDrivetrain) {
    // Save parameters to use in other methods below
    m_percent = percent;
    m_duration = time * 1000;
    m_drive = romiDrivetrain;

    // Add requirements for all subsystems being used by this command
    addRequirements(romiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Save the time when the command started running (now)
    m_startTime = System.currentTimeMillis();
    DataLogUtil.writeMessageEcho("DriveTime init ");
    System.out.println("STARTING DRIVE TIME");

    // Dirve at desired speed
    m_drive.arcadeDrive(m_percent, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Need to continually call arcadeDrive to feed the motor watchdog
    m_drive.arcadeDrive(m_percent, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Turn off the motors
    m_drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
