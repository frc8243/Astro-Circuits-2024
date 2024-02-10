
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  // private final static AddressableLED LED = new AddressableLED(1);
  // private final static AddressableLEDBuffer LEDBuffer = new
  // AddressableLEDBuffer(60);
  private Spark m_blinkin = new Spark(2);
  private static double color;
  private static double idleColor;

  /** Creates a new LEDs. */
  public LEDs() {
    /*
     * This section of code sets the LED color to the current alliance color, if we
     * are not connected to field, or there is no alliance, it is purple.
     */
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        color = 0.61;
      } else {
        color = 0.87;
      }
    } else {
      color = 0.89;
    }
    idleColor = color;
  }

  @Override
  public void periodic() {
    m_blinkin.set(color);
    // LED.setData(LEDBuffer);
  }

  public static void noteReady() {
    color = 0.77;
  }

  /**
   * 
   * @param location Where do we want the piece, 1 for claw, 2 for shooter
   */
  public static void askForNote(int location) {
    if (location == 1) {
      color = 0.65;
    } else if (location == 2) {
      color = 0.93;
    }
  }

  public static void trackingTag() {
    color = -0.57;
  }

  public static void returnToIdle() {
    color = idleColor;
  }

}
