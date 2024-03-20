package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface LEDIO {
    public void noteReady();

    public void updateIdle(Alliance alliance);

    public void askForNote(int location);

    public void trackingTag();

    public void returnToIdle();

    public void readyToShoot();

    public void disabledIdle();

    public void periodicLoop();
}
