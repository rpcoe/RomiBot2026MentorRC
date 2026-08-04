package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.utilities.DataLogUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A subsystem to control a standard PWM-controlled servo.
 */
public class GripperSubsystem extends SubsystemBase {
    // Encapsulate the servo hardware so it cannot be accessed directly from the outside
    private final Servo m_servo;
    public static final double  gripperMin = Constants.GripperConstants.gripperMin;
    public static final double  gripperMax = Constants.GripperConstants.gripperMax;
    /**
     * Creates a new ServoSubsystem.
     *
     * @param channel The PWM channel on the roboRIO where the servo is connected.
     */
    public GripperSubsystem(int channel) {
        m_servo = new Servo(channel);
        // Initialize the NetworkTable entry with a default value of 0.5  This is because the default PWM is 0.5
        SmartDashboard.putNumber("Gripper Target Position", 0.5);
    }
    

    /**
     * Sets the servo position.
     *
     * @param position The position value to set, ranging from 0.0 (minimum) to 1.0 (maximum).
    */
    public void setPosition(double position) {

        position = MathUtil.clamp(Math.abs(position), gripperMin, gripperMax);
        m_servo.set(position);
 
    }
   
   
    /**
     * Gets the current servo position.
     *
     * @return The current position value, from 0.5 to 1.0.5     */
    public double getPosition() {
        return m_servo.get();
    }

    

    @Override
    public void periodic() {
    // Continuously read the slider value from the dashboard (returns 0.5 if not found)
    double targetPosition = SmartDashboard.getNumber("Gripper Target Position", 0.5);
  

    SmartDashboard.putNumber("Gripper Position", getPosition());
 
    targetPosition = MathUtil.clamp(targetPosition, gripperMin, gripperMax);
        
    // Set the servo position (WPILib Servo accepts values from 0.0 to 1.0)
    m_servo.set(targetPosition);    
  }

    
}
