
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  // private final static AddressableLED LED = new AddressableLED(1);
  // private final static AddressableLEDBuffer LEDBuffer = new
  // AddressableLEDBuffer(60);
  Spark m_blinkin = new Spark(2);

  /** Creates a new LEDs. */
  public LEDs() {

    // LED.allLEDS(100, 50, 200);
  }

  @Override
  public void periodic() {
    m_blinkin.set(0.61);
    // LED.setData(LEDBuffer);
  }

}
