// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.utilities.DataLogUtil;

public class AutoGrab extends SequentialCommandGroup {

  /** 
   * Creates a new Autonomous routine to grab an object. This will drive out for a specified distance,
   * grab the object, turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param gripper The gripper subsystem on which this command will run
   */
  public AutoGrab(RomiDrivetrain drivetrain, GripperSubsystem gripper) {
    addCommands(
        new GripperSet(gripper, 0),
        new DriveTime(0.4, 1, drivetrain),
        new GripperSet(gripper, 0.9),
        new DriveTime(0, 0.25, drivetrain),   // pause to allow gripper to close
        new DriveTime(-0.4, 0.8, drivetrain),
        new GripperSet(gripper, 0),
        new DriveTime(-.20, 0.5, drivetrain)   

    );
  }
}