// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.utilities.DataLogUtil;

public class DriveWithJoystick extends Command {

  // Parameters from constructor
  private final Joystick joystick;
  private final RomiDrivetrain romiDrivetrain;  // Drivetrain subsystem

  /**
   * Drives the robot using the joystick in arcade drive mode.  Y axis = move forward/back,
   * X axis = rotate.
   * @param joystick Joystick to use for driving
   * @param romiDrivetrain
   */
  public DriveWithJoystick(Joystick joystick, RomiDrivetrain romiDrivetrain) {
    // Save parameters to use in other methods below
    this.joystick = joystick;
    this.romiDrivetrain = romiDrivetrain;

    // Add requirements for all subsystems being used by this command
    addRequirements(romiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("DriveWithJoystick:  Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // romiDrivetrain.arcadeDrive(-joystick.getY(), -joystick.getX(), false);

    //double xSpeed = MathUtil.applyDeadband(-joystick.getY(), OIConstants.joystickDeadband);
    //double zRotation = MathUtil.applyDeadband(-joystick.getX(), OIConstants.joystickDeadband);
    double xSpeed  = 0.0;   //I am not using the joystick for now, so set to 0.0
    double zRotation = 0.0;

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    double leftSpeed = xSpeed - zRotation*(1-Math.abs(xSpeed)*0.7);
    double rightSpeed = xSpeed + zRotation*(1-Math.abs(xSpeed)*0.7);

    // Find the maximum possible value of (throttle + turn) along the vector
    // that the joystick is pointing, then desaturate the wheel speeds
    double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
    double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
    if (greaterInput > 0.0) {
      double saturatedInput = (greaterInput + lesserInput) / greaterInput;
      leftSpeed /= saturatedInput;
      rightSpeed /= saturatedInput;
    }

    romiDrivetrain.setLeftMotorOutput(leftSpeed);
    romiDrivetrain.setRightMotorOutput(rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Turn off the motors
    romiDrivetrain.stopMotors();
    DataLogUtil.writeMessage("DriveWithJoystick:  End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
