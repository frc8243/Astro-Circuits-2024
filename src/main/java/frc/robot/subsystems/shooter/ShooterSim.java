package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim implements ShooterIO {
    private FlywheelSim shooterWheel;
    private FlywheelSim feedWheel;

    public ShooterSim() {

    }

    public void setFeedMotor(double speed) {

    }

    public void setShootMotor(double speed) {
    }

    public void stop() {
    }

    public double getFeedSpeed() {
        return 0;
    }

    public double getShootSpeed() {
        return 0;
    }

    public void spinShootMotor(double rpm) {

    }

    public void spinFeedMotor(double rpm) {

    }

    @Override
    public void loadCHRPfile(String file) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'loadCHRPfile'");
    }

    @Override
    public void play() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'play'");
    }

    @Override
    public void pause() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'pause'");
    }
}
