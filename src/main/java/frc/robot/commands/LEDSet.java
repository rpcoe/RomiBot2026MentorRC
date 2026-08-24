// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utilities.DataLogUtil;


public class LEDSet extends Command {
  private final LED led;
  private final LEDColor color;
  private final boolean turnOn;

  // Available LED colors on the Romi
  public enum LEDColor {
    YELLOW,
    RED
  }

  /**
   * Turns one of the Romi LEDs either on or off
   * @param color Which LED to turn on/off.  See the {@link LEDColor} enum for available colors.
   * @param turnOn true = on, false = off
   * @param led LED subsystem
   */
  public LEDSet(LEDColor color, boolean turnOn, LED led) {
    // Save parameters for use in command
    this.led = led;
    this.color = color;
    this.turnOn = turnOn;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (color) {
      case YELLOW:
        led.setYellowLed(turnOn);
        DataLogUtil.writeMessage("LEDSet:  Init, Yellow =", turnOn);
        break;
      case RED:
        led.setRedLed(turnOn);
        DataLogUtil.writeMessage("LEDSet:  Init, Red =", turnOn);
        break;
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
