package frc.robot.subsystems.climber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
//import edu.wpi.first.wpilibj.simulation.ClimberSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utils.SimEncoder;

public class ClimberSim implements ClimberIO {
    public static SimEncoder climberSimEncoder; // Our own class. Just a dumb class that gets and sets values
                                                // representing an encoder
    public static ElevatorSim climberSim; // from WPILib
    public double climberSpeed;
    public final static DCMotor climberGearbox = DCMotor.getNEO(1);
    // Simulated climber constants and gearbox
    public static final double climberGearRatio = 9.0;
    public static final double climberDrumRadius = Units.inchesToMeters(1.0);
    public static final double climberCarriageMass = 5.5; // kg
    public static final double climberEncoderDistPerPulse = 2.0 * Math.PI * climberDrumRadius / 4096;

    public ClimberSim() {
        climberSimEncoder = new SimEncoder("climber");
        climberSim = new ElevatorSim(
                climberGearbox,
                climberGearRatio,
                climberCarriageMass,
                climberDrumRadius,
                Constants.ClimberConstants.MIN_CLIMBER_HEIGHT,
                Constants.ClimberConstants.MAX_CLIMBER_HEIGHT,
                true, 0, // whether to use gravity
                VecBuilder.fill(Constants.ClimberConstants.simMeasurementStdDev));

    }

    @Override
    public double getEncoderSpeed() {
        return climberSimEncoder.getSpeed();
    }

    @Override
    public void setMotorSpeed(double speed) {
        climberSpeed = speed;
    }

    @Override
    public double getEncoderPosition() {
        return climberSimEncoder.getDistance();
    }

    @Override
    public void setEncoderPosition(double position) {
        climberSimEncoder.setDistance(position);
    }

    @Override
    public double getClimberCurrent() {
        return climberSim.getCurrentDrawAmps();
    }

    @Override
    public void periodicUpdate() {
        // sets input for climber motor in simulation
        SmartDashboard.putNumber("Climber/Sim motor speed (-1 to 1)", climberSpeed);
        climberSim.setInput(climberSpeed * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Climber/Sim battery voltage", RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        climberSim.update(0.02);
        // Finally, we set our simulated encoder's readings
        SmartDashboard.putNumber("Climber/Sim position (m)", climberSim.getPositionMeters());

        climberSimEncoder.setDistance(climberSim.getPositionMeters());

        SmartDashboard.putNumber("Climber/Sim encoder position (m)", climberSimEncoder.getDistance());

        // sets our simulated encoder speeds
        climberSimEncoder.setMotorSpeed(climberSim.getVelocityMetersPerSecond());

        // BatterySim estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));
    }
}