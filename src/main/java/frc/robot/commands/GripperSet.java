// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utilities.DataLogUtil; 
public class GripperSet extends Command {
private final GripperSubsystem m_gripperSubsystem;
private final double m_position;

  /**                             
   * Creates a new ServoAngle command.
   *
   * @param subsystem The Servo subsystem used by this command. n
   * @param angle The target angle in 0.0 to 1.0.
   */
  public GripperSet(GripperSubsystem subsystem, double angle) {
    m_gripperSubsystem = subsystem;
    m_position = angle;
    
    // Declare subsystem dependency to prevent conflicting commands
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Gripper Target Position", m_position);
    m_gripperSubsystem.setPosition(m_position);
    DataLogUtil.writeMessageEcho("GripperSet init ");
    System.out.println("STARTING GRIPPER SET");

  
  }

   // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripperSubsystem.setPosition(0.5);   // This should be to initial position in constants
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Returns true so that it runs once and ends, otherwise it will keep running and not allow other commands to run
    return true; 
  }
}
































































































































































































































































































































































































