
package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.rollerclaw.RollerClaw;
import frc.robot.subsystems.shooter.Shooter;

@SuppressWarnings("unused")
public class LEDs extends SubsystemBase {
  private static LEDIO ledIO;

  private static boolean ledsActioned;
  private static String ledState;

  /** Creates a new LEDs. */
  public LEDs(LEDIO io) {
    ledIO = io;
    ledState = "Booting";
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("DriverAssists/LEDState", ledState);
    ledIO.periodicLoop();
    if (RollerClaw.getNoteStatus() || Shooter.getNoteStatus()) {
      noteReady();
      ledsActioned = false;
      if (Shooter.getShooterSpeed() >= Shooter.getTargetRPM()) {
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
    ledState = "Note Ready";
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
    ledState = (location == 1) ? "Requesting Claw" : "Requesting Shooter";
  }

  public void trackingTarget() {
    ledsActioned = true;
    ledIO.trackingTarget();
    ledState = "Tracking Target";
  }

  public void linedUp() {
    ledsActioned = true;
    ledIO.linedUp();
    ledState = "Lined Up";
  }

  public void returnToIdle() {
    ledIO.returnToIdle();
    ledState = "Idle";
  }

  public void readyToShoot() {
    ledIO.readyToShoot();
    ledState = "Ready to Shoot";
  }

  public void disabledIdle() {
    ledIO.disabledIdle();
    ledState = "Disabled Idle";
  }

}
