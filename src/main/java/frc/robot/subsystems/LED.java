// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.LEDConstants.*;

public class LED extends SubsystemBase {
  // Note that the Romi can use the OnBoardIO class instead of this subsystem to access
  // both the LEDs and the pushbuttons.  This LED class is for instructional purposes.

  // DataLog variables
  private final int logRotationKey;         // key for the logging cycle for this subsystem
  private final DataLog log = DataLogManager.getLog();
  private final BooleanLogEntry dLogYellow = new BooleanLogEntry(log, "/LED/YellowIsOn");
  private final BooleanLogEntry dLogRed = new BooleanLogEntry(log, "/LED/RedIsOn");
  private final BooleanLogEntry dLogGreen = new BooleanLogEntry(log, "/LED/GreenIsOn");


  // LED objects
  private final DigitalOutput ledYellow = new DigitalOutput(dioYellowLED);
  private final DigitalOutput ledRed = new DigitalOutput(dioRedLED);
  private final DigitalOutput ledGreen = new DigitalOutput(dioGreenLED);

  // LED tracking variables
  boolean yellowIsOn = false;
  boolean redIsOn = false;
  boolean greenIsOn = false;

  /**
   * Creates the LED subsystem.
   */
  public LED() {
    // Note that the Romi can use the OnBoardIO class instead of this subsystem to access
    // both the LEDs and the pushbuttons.  This LED class is for instructional purposes.

    logRotationKey = DataLogUtil.allocateLogRotation();  // Get log rotation for this subsystem

    // Initialize LEDs
    setYellowLed(false);
    setRedLed(false);
    setGreenLed(false);
    // Prime the DataLog to reduce delay when first enabling the robot
    updateLog(true);
  }

  /**
   * Sets the yellow LED.
   *
   * @param turnOn true = turn the Yellow LED on, false = turn the Yellow LED off
   */
  public void setYellowLed(boolean turnOn) {
    ledYellow.set(turnOn);
    yellowIsOn = turnOn;
  }

  /**
   * Sets the red LED.
   *
   * @param turnOn true = turn the Red LED on, false = turn the Red LED off
   */
  public void setRedLed(boolean turnOn) {
    ledRed.set(turnOn);
    redIsOn = turnOn;
  }

  /**
   * Sets the green LED.
   *
   * @param turnOn true = turn the Green LED on, false = turn the Green LED off
   */
  public void setGreenLed(boolean turnOn) {
    ledGreen.set(turnOn);
    greenIsOn = turnOn;
  }

  /*returns the state of the yellow LED  */
  public boolean getYellowLed() {
    return yellowIsOn;
  }

  /*returns the state of the red LED  */
  public boolean getRedLed() {
    return redIsOn;
  }

  /*returns the state of the green LED  */
  public boolean getGreenLed() {
    return greenIsOn;
  }

  /**
   * Write information about the LEDs to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogYellow.append(yellowIsOn, timeNow);
      dLogRed.append(redIsOn, timeNow);
      dLogGreen.append(greenIsOn, timeNow);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(DataLogUtil.isMyLogRotation(logRotationKey)) {
      // Update data on SmartDashboard  
      SmartDashboard.putBoolean("LED Yellow", yellowIsOn);
      SmartDashboard.putBoolean("LED Red", redIsOn);
      SmartDashboard.putBoolean("LED Green", greenIsOn);

      // Update filelog
      updateLog(false);
    }
  }
}
