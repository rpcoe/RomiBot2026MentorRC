// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.commands.LEDSet.LEDColor;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot subsystems  
  private final RomiDrivetrain romiDrivetrain = new RomiDrivetrain();
  private final GripperSubsystem m_gripper = new GripperSubsystem(Constants.GripperConstants.kServo);
  private final LED led = new LED();
  //private final GripperSubsystem m_servo = new GripperSubsystem(Constants.GripperConstants.kServo);
  // Define other utilities

  // Define controllers
  // private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  // private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  // private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  
  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DataLogUtil.writeMessageEcho("RobotContainer: Constructor, Version =", Constants.bcrRobotCodeVersion);

    // Configure the various bindings
    //configureButtonBindings();
    configureDashboard();

    // Set any default commands for subsystems
    romiDrivetrain.setDefaultCommand( new DriveWithJoystick(leftJoystick, romiDrivetrain) );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  
  private void configureButtonBindings() {
    // Create joystick buttons
    // Note that joystick button numbering starts at 1 for JoystickButton(,)
    JoystickButton[] leftJoystickButtons = new JoystickButton[3];
    for (int i = 1; i < leftJoystickButtons.length; i++) {
      leftJoystickButtons[i] = new JoystickButton(leftJoystick, i);
    }

    // Assign commands to joystick buttons
    leftJoystickButtons[1].onTrue( new DriveTime(0.5, 1.0, romiDrivetrain) );
    // leftJoystickButtons[2].onTrue( new DriveTime(0.5, 1.0, romiDrivetrain) );
  }
    

  /**
   * Configures dashboard controls.
   */
  private void configureDashboard() {
    // auto selection widget
    m_chooser.setDefaultOption("Auto Routine Time", new  DriveTime(0.3, 1, romiDrivetrain));
    m_chooser.addOption("Auto Grab", new AutoGrab(romiDrivetrain, m_gripper));
    m_chooser.addOption("Auto Drive a Square", new AutonomousSquare(romiDrivetrain ));
    

    SmartDashboard.putData(m_chooser);
  
    // Driving commands
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(romiDrivetrain));
    SmartDashboard.putData("Drive 3 sec", new DriveTime(0.5, 3, romiDrivetrain));
    SmartDashboard.putData("Drive Turn", new DriveTurn(DriveTurn.TurnMode.GYRO_ABSOLUTE, romiDrivetrain));
    SmartDashboard.putData("AutonomousDistance", new AutonomousSquare(romiDrivetrain));
    SmartDashboard.putData("AutoGrab", new AutoGrab(romiDrivetrain, m_gripper));
    // LED commands
    SmartDashboard.putData("LED Yellow on", new LEDSet(LEDColor.YELLOW, true, led));
    SmartDashboard.putData("LED Yellow off", new LEDSet(LEDColor.YELLOW, false, led));
    SmartDashboard.putData("LED Red on", new LEDSet(LEDColor.RED, true, led));
    SmartDashboard.putData("LED Red off", new LEDSet(LEDColor.RED, false, led));
  
    //  GRIPPER commands

    SmartDashboard.putData("Gripper Open", new GripperSet(m_gripper, 0.0));   //These are extremes. Acturals will be determined by gripperMax and gripperMin in Constants.GripperConstants
    SmartDashboard.putData("Gripper Close", new GripperSet(m_gripper, 1.0));
    SmartDashboard.putData("Gripper Test", new GripperTest(m_gripper));

  }

  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return m_chooser.getSelected(); 
  }

  // *************** Init and periodic methods

   /**
   * Method called when robot is initiated.
   */
  public void robotInit() {
  }

  /**
   * Method called once every scheduler cycle, regardless of
   * whether robot is in auto/teleop/disabled mode.
   */
  public void robotPeriodic() {
    DataLogUtil.advanceLogRotation();
  }

  /**
   * Method called when robot is disabled.  Method is also 
   * called after robot is initially booted up, since it boots
   * up into Disabled mode.
   */
  public void disabledInit() {
    DataLogUtil.writeMessageEcho("Disabled: Robot disabled");

    romiDrivetrain.zeroGyroRotation();
    romiDrivetrain.zeroEncoders();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
  }
  
  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    m_gripper.setPosition(0.5);
    DataLogUtil.writeMessageEcho("Teleop: Mode Init");
  }

   /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {}
 
  /**
   * Method called when autonomouse mode is initialized/enabled.
   */
  public void autonomousInit() {
    DataLogUtil.writeMessageEcho("Auto: Mode Init ");
  }

   /**
   * Method called once every scheduler cycle when autonomous mode is initialized/enabled.
   */
  public void autonomousPeriodic() {}
}
