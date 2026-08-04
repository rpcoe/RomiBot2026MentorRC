// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String bcrRobotCodeVersion = "A1";

    public static final class OIConstants {
      // Standard ports used for BCR Driver Station
      public static final int usbXboxController = 0;
      public static final int usbLeftJoystick = 1;
      public static final int usbRightJoystick = 2;
      public static final int usbCoPanel = 3;

      public static final double joystickDeadband = 0.04;
    }

    public static final class LEDConstants {
        // Note - green LED should be at DIO address = 1
        public static final int dioRedLED = 2;
        public static final int dioYellowLED = 3;
    }

    public static final class DriveConstants{
        // The Romi has the left and right motors set to
        // PWM channels 0 and 1 respectively
        public static final int leftDriveMotor = 0;
        public static final int rightDriveMotor = 1;
 
        // The Romi has onboard encoders that are hardcoded
        // to use DIO pins 4/5 and 6/7 for the left and right
        public static final int leftEncoderA = 4;
        public static final int leftEncoderB = 5;
        public static final int rightEncoderA = 6;
        public static final int rightEncoderB = 7;
 
        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterInch = 2.7; // 70 mm
        /* The Standard Romi Chassis found here, 
           https://www.pololu.com/category/203/romi-chassis-kits,
           has a wheel placement diameter (149 mm) - width of the 
           wheel (8 mm) = 141 mm or 5.551 inches. We then take into 
           consideration the width of the tires.
        */    
        public static final double kWheelSpacingInch = 5.5;    // Calibrated on Romi-6 * 1.048 by turning in place.  Need to calibrate after calibrating wheel diameter.
        public static final double kTurnInchPerDegree = Math.PI * kWheelSpacingInch  / 360;    // approx. 0.1 inch/deg.  Need to convert distance travelled to degrees. 
        //  public static double rightDriftCorrection = 0.957;      // needed to correct forward drift to the right

        // Drive turning constants
        public static final double kP_angle = .003;         //  .003 OK for turn Gyro
        public static final double kP_angleEnc = 0.005;         //  .005 OK for turn Encoders
        public static final double kS_angle = 0.10;       // Min wheel speed (pct voltage) for turning
      }

        // Gripper constants
      public static final class GripperConstants {
          public static final int kServo = 3;   
          public static final double gripperMin = 0.3;   
          public static final double gripperMax = 0.6; //  0.65 on Romi-2,  0.6 nominally
      } 
}
