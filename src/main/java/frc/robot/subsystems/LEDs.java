
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  // private final static AddressableLED LED = new AddressableLED(1);
  // private final static AddressableLEDBuffer LEDBuffer = new
  // AddressableLEDBuffer(60);
  Spark m_blinkin = new Spark(2);
  DigitalInput photoSensor = new DigitalInput(0);
  double value = -1;
  DigitalInput ledLimitSwitchShooter = new DigitalInput(LEDConstants.kOutLimitSwitch);

  /** Creates a new LEDs. */
  public LEDs() {

    // LED.allLEDS(100, 50, 200);
  }

  @Override
  public void periodic() {
    if (!ledLimitSwitchShooter.get()) {
      System.out.println("limit switch on");
      m_blinkin.set(0.75);
      // m_blinkin.set(0.35);
    } else if (photoSensor.get()) {
      m_blinkin.set(-0.01);
    } else {
      System.out.println("limit switch off");

      m_blinkin.set(0.93);
    }

    // LED.setData(LEDBuffer);
  }

}
