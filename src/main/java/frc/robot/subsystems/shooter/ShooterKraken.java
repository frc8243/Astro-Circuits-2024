package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;

public class ShooterKraken implements ShooterIO {
    private static TalonFX shootMotor = new TalonFX(ShooterConstants.kShootMotorID, "rio");
    private static TalonFX feedMotor = new TalonFX(ShooterConstants.kFeedMotorID, "rio");
    private DutyCycleOut shootRequest = new DutyCycleOut(0.0);
    private DutyCycleOut feedRequest = new DutyCycleOut(0.0);

    public ShooterKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.withSupplyCurrentLimit(80);
        config.CurrentLimits.withSupplyCurrentLimitEnable(true);
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        shootMotor.getConfigurator().apply(config);
        feedMotor.getConfigurator().apply(config);

    }

    @Override
    public void setFeedMotor(double speed) {
        feedRequest.Output = speed;
        feedMotor.setControl(feedRequest);
    }

    @Override
    public void setShootMotor(double speed) {
        shootRequest.Output = speed;
        shootMotor.setControl(shootRequest);
    }

    @Override
    public void stop() {
        feedMotor.stopMotor();
        shootMotor.stopMotor();
    }

    @Override
    public void spinShootMotor(double rpm) {
        
    }

    @Override
    public void spinFeedMotor(double rpm) {
        
    }

    @Override
    public double getFeedSpeed() {
        return feedMotor.getVelocity().refresh().getValueAsDouble();
    }

    @Override
    public double getShootSpeed() {
        return shootMotor.getVelocity().refresh().getValueAsDouble();
    }

}
