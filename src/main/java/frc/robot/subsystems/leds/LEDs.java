
package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.subsystems.rollerclaw.RollerClaw;
import frc.robot.subsystems.shooter.Shooter;

@SuppressWarnings("unused")
public class LEDs extends SubsystemBase {
  private static LEDIO ledIO;
  private static boolean ledsActioned;

  /** Creates a new LEDs. */
  public LEDs(LEDIO io) {
    ledIO = io;
  }

  @Override
  public void periodic() {
    ledIO.periodicLoop();
    if (RollerClaw.getNoteStatus() || Shooter.getNoteStatus()) {
      noteReady();
      ledsActioned = false;
      if (Shooter.getShooterSpeed() >= NeoMotorConstants.kFreeSpeedRpm) {
        readyToShoot();
      }
    } else if (ledsActioned == false) {
      returnToIdle();
    }
    if (DriverStation.isDisabled()) {
      disabledIdle();
    }
  }

  public void noteReady() {
    ledIO.noteReady();
  }

  public void updateIdle(Alliance alliance) {
    ledIO.updateIdle(alliance);
  }

  /**
   * 
   * @param location Where do we want the piece, 1 for claw, 2 for shooter
   */
  public void askForNote(int location) {
    ledsActioned = true;
    ledIO.askForNote(location);
  }

  public static void trackingTag() {
    ledsActioned = true;
    ledIO.trackingTag();
  }

  public void returnToIdle() {
    ledIO.returnToIdle();
  }

  public void readyToShoot() {
    ledIO.readyToShoot();
  }

  public void disabledIdle() {
    ledIO.disabledIdle();
  }

}
