// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utilities.DataLogUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleLed extends Command {
  private final LED led;

  /** Creates a new ToggleLed. 
   * @param led The LED subsystem this command will run on.
   * */
  public ToggleLed(LED led) {
    this.led = led;
    addRequirements(led);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessageEcho("ToggleLed: initialize" + " - Current Green LED State: " + led.getGreenLed());
    if ( led  .getGreenLed()) {
      led.setGreenLed(false);
    } else {
      led .setGreenLed(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
