package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.utils.PIDUtil;

public class Climber extends ProfiledPIDSubsystem {

    private ClimberIO climberIO;

    private static final Constraints constraints = new Constraints(ClimberConstants.kMaxVel,
            ClimberConstants.kMaxAccel);

    private final ClimberVisualizer climberVisualizer = new ClimberVisualizer();

    public Climber(ClimberIO io) {
        super(new ProfiledPIDController(ClimberConstants.kpPos, ClimberConstants.kiPos, ClimberConstants.kdPos,
                constraints));

        getController()
                .setTolerance(ClimberConstants.kHeightTolerance, ClimberConstants.kVelocityTolerance);

        climberIO = io;
        climberIO.setEncoderPosition(0.0);
        this.setGoal(0.0);
        this.enable();
    }

    @Override
    public void periodic() {
        // inside this next line, this is called
        // useOutput(m_controller.calculate(getMeasurement()),
        // m_controller.getSetpoint())
        // So what is the output of calculate?
        // SmartDashboard.putNumber("Climber/Profiled PID setpoint pos",
        // getController().calculate(getMeasurement(), getController().getSetpoint()));

        super.periodic();
        climberIO.periodicUpdate();
        double currentPos = getMeasurement();// A
        double currentVel = getEncoderSpeed();
        SmartDashboard.putNumber("Climber/Profiled goal (m)", getGoal());
        SmartDashboard.putNumber("Climber/position (m)", currentPos);
        SmartDashboard.putNumber("Climber/velocity (m per s)", currentVel);
        SmartDashboard.putBoolean("Climber/Profiled subsytem enabled", this.isEnabled());
        SmartDashboard.putNumber("Climber/Profiled PID setpoint pos", getController().getSetpoint().position);
        SmartDashboard.putNumber("Climber/Profiled PID position error", getController().getPositionError());
        SmartDashboard.putNumber("Climber/Profiled PID velocity error", getController().getVelocityError());
        SmartDashboard.putNumber("Climber/Profiled PID position tol", getController().getPositionTolerance());
        SmartDashboard.putNumber("Climber/Profiled PID velocity tol", getController().getVelocityTolerance());
        SmartDashboard.putNumber("Climber/Profiled PID goal pos", getController().getGoal().position);
        SmartDashboard.putBoolean("Climber/Profiled PID position at Goal", getController().atGoal());
        SmartDashboard.putBoolean("Climber/Profiled PID position at set point", getController().atSetpoint());

        climberVisualizer.update(currentPos);
    }

    /**
     * Method to wrap the enabling/goal setting to simplify
     * RobotContainer.configureBindings()
     * 
     * @param goalHeight Height in meters to set the climber to
     * @return Command for use with a button press
     */
    public Command setClimberHeight(double goalHeight) {
        return this.runOnce(
                () -> {
                    this.enable();
                    this.setGoal(goalHeight);
                });
    }

    // returns height the climber is at. Required to override this
    @Override
    public double getMeasurement() {
        return climberIO.getEncoderPosition();
    }

    // returns speed of climber
    public double getEncoderSpeed() {
        return climberIO.getEncoderSpeed();
    }

    public void setMotorSpeed(double speed) {
        SmartDashboard.putNumber("Climber/motor speed (-1 to 1)", speed);
        climberIO.setMotorSpeed(speed);
    }

    public void setMotorSpeedGravityCompensation(double speed) {
        climberIO.setMotorSpeed(speed + ClimberConstants.kGravityCompensation);
    }

    public double getClimberCurrent() {
        return climberIO.getClimberCurrent();
    }

    // required to override this
    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Climber/Profiled useOutput output var (-1 to 1)", output);
        SmartDashboard.putNumber("Climber/Profiled setpoint position (m)", setpoint.position);
        SmartDashboard.putNumber("Climber/Profiled setpoint velocity (m per s)", setpoint.velocity);
        SmartDashboard.putNumber("Climber/Profiled setpoint position error (m)", setpoint.position - getMeasurement());
        SmartDashboard.putNumber("Climber/Profiled setpoint velocity error (m)",
                setpoint.velocity - getEncoderSpeed());

        // not doing anything with the position error ??

        // Calculate the feedforward from the setpoint
        double speed = ClimberConstants.kFeedForward * setpoint.velocity;
        // accounts for gravity in speed
        speed += ClimberConstants.kGravityCompensation;

        speed += output;

        climberIO.setMotorSpeed(speed);
    }

    // @Override
    // protected double getMeasurement() {
    // return climberIO.getEncoderPosition();
    // }

    public double getGoal() {
        return m_controller.getGoal().position;
    }

    // Checks to see if climbers are within range of the setpoints
    public boolean atGoal() {
        return (PIDUtil.checkWithinRange(getGoal(), getMeasurement(), ClimberConstants.kHeightTolerance));
    }

    public void setEncoderPosition(double position) {
        climberIO.setEncoderPosition(position);
    }

    /**
     * Factory Method to return a command for manual climber control
     * 
     * @param speed Percent to run the motor with
     * @return Command to be called with a Trigger
     */
    public Command getClimberCommand(double speed) {
        return this.startEnd(
                () -> {
                    this.disable();
                    climberIO.setMotorSpeed(speed);

                },
                () -> {
                    climberIO.setMotorSpeed(0);
                });
    }
}
