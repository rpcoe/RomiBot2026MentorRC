// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utilities.DataLogUtil;

public class GripperTest extends Command {
  private final GripperSubsystem m_gripperSubsystem;
  
  public static final double  gripperMin = Constants.GripperConstants.gripperMin;
  public static final double  gripperMax = Constants.GripperConstants.gripperMax;
  public static  double  GyroAngle1 = 999.0;   // start out of range so it doesn't trigger a false "lost WIFI" message on first loop
  public static  double  GyroAngle51 = 999.0;   // start out of range so it doesn't trigger a false "lost WIFI" message on first loop
private int count = 0;
  private int cycle = 0;

  public GripperTest(GripperSubsystem subsystem) {
    m_gripperSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    
    count = 0;
    cycle = 0;
          DataLogUtil.writeMessageEcho("Gripper cycle test started");

  }

  @Override
  public void execute() {
    count++;

    // First half of cycle (1 second / 50 loops @ 20ms): go to min
    // Second half of cycle (1 second / 50 loops @ 20ms): go to max
    if(count == 1) {
      m_gripperSubsystem.setPosition(gripperMin);
      SmartDashboard.putNumber("Gripper Target Position", gripperMin);
      GyroAngle1 = SmartDashboard.getNumber("Drive Gyro Angle", 0.0);


    } else if(count==51){
      m_gripperSubsystem.setPosition(gripperMax);
      SmartDashboard.putNumber("Gripper Target Position", gripperMax);
      GyroAngle51 = SmartDashboard.getNumber("Drive Gyro Angle", 0.0);

    }
          
    if (count >= 100) {
      count = 0;
      cycle++;
      DataLogUtil.writeMessageEcho("Gripper cycle test", "Cycle =", cycle); 
  
    }

    // Pass the calculated targetPosition to the subsystem (NOT m_position)
    //m_gripperSubsystem.setPosition(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    //m_gripperSubsystem.setPosition(0.5); // Reset to neutral position
  }

  // We use the fact that the gyro angle drifts over time to detect if the robot has lost WIFI and the test is no longer running.  If the gyro angle is the same at the start and end of a cycle, we assume that the robot has lost WIFI and the test is aborted.
  // This is a hack, but it works for this test.  In a real robot, we would use a more robust method to detect if the robot has lost WIFI.
  @Override
  public boolean isFinished() {

    if(GyroAngle1 == GyroAngle51) {
      DataLogUtil.writeMessageEcho("Gripper cycle test", "Lost WIFI, test aborted");
      return true;
    }
    if (cycle >= 1000) {
      DataLogUtil.writeMessageEcho("Gripper cycle test", "Cycle =", cycle, "Test complete");   
      return true;
    }

    return false; 
  }
}
