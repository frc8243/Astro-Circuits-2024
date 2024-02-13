package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ClimberVisualizer {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d climberMechanism;

    public ClimberVisualizer() {
        mech = new Mechanism2d(1, 1);
        root = mech.getRoot("root", 0.3, 0);
        climberMechanism = root
                .append(new MechanismLigament2d("climber", Constants.ClimberConstants.kMinClimberHeight, 50));
        climberMechanism.setColor(new Color8Bit(0, 204, 255));
        climberMechanism.setLineWeight(20);
        SmartDashboard.putData("Mech2d", mech);
    }

    public void update(double currentPos) {
        climberMechanism.setLength(Constants.ClimberConstants.kMinClimberHeight + currentPos);
    }
}