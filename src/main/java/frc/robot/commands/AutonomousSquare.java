// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.utilities.DataLogUtil;
//import frc.robot.utilities.FileLog;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousSquare extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param log  
   */
  public AutonomousSquare(RomiDrivetrain drivetrain) {
    addCommands(
        new DriveTime(0.4, 1, drivetrain),
        new DriveTurn( DriveTurn.TurnMode.GYRO_RELATIVE, drivetrain),
        new DriveTime(0.4, 1, drivetrain),
        new DriveTurn( DriveTurn.TurnMode.GYRO_RELATIVE, drivetrain),
        new DriveTime(0.4, 1, drivetrain),
        new DriveTurn( DriveTurn.TurnMode.GYRO_RELATIVE, drivetrain),
        new DriveTime(0.4, 1, drivetrain),
        new DriveTurn( DriveTurn.TurnMode.GYRO_RELATIVE, drivetrain)
       
      );  

        
  }
}
