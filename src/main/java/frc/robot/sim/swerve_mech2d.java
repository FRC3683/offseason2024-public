package frc.robot.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utils.controls.swerve_kin2;
import frc.robot.utils.controls.swerve_kin2.module_state;

public class swerve_mech2d {

    public swerve_mech2d(swerve_kin2 kin, double display_size) {
        num_modules = kin.num_modules;
        module_translations = kin.module_mount_positions;
        max_module_speed = kin.max_module_mps;
        mech = new Mechanism2d(display_size, display_size);
        root = mech.getRoot("swerve", display_size/2.0, display_size/2.0);
        rotation = root.append(new MechanismLigament2d("rotation", 0, 0));
        visual_rotation = rotation.append(new MechanismLigament2d("arrow", 0.5, 0.0, 6.0, new Color8Bit(Color.kGreen)));
        visual_rotation.append(new MechanismLigament2d("head1", 0.15, 135.0, 4.0, new Color8Bit(Color.kGreen)));
        visual_rotation.append(new MechanismLigament2d("head2", 0.15, -135.0, 4.0, new Color8Bit(Color.kGreen)));
        desired_module_states = new MechanismLigament2d[num_modules];
        actual_module_states = new MechanismLigament2d[num_modules];
        for(int i = 0; i < num_modules; ++i) {
            final Translation2d trans = module_translations[i];
            desired_module_states[i] = make_module(trans, "desired_", i, new Color8Bit(Color.kOrange));
            actual_module_states[i] = make_module(trans, "actual_", i, new Color8Bit(Color.kYellow));
        }
    }

    final Color8Bit black = new Color8Bit(Color.kBlack);
    private MechanismLigament2d make_module(Translation2d trans, String prefix, int i, Color8Bit color) {
        var offset = rotation.append(new MechanismLigament2d(prefix+"offset"+i, trans.getNorm(), trans.getAngle().getDegrees(), 0, black));
        var module_state = offset.append(new MechanismLigament2d(prefix+"state"+i, 0.1, 0.0, 6.0, color));
        module_state.append(new MechanismLigament2d(prefix+"state"+i+"_i", 0.15, 135.0, 4.0, color));
        module_state.append(new MechanismLigament2d(prefix+"state"+i+"_ii", 0.15, -135.0, 4.0, color));
        return module_state;
    }

    public void init() {
        SmartDashboard.putData("swerve_mechanism2d", mech);
    }

    public void update(Rotation2d pose_rotation, module_state[] desired_states, module_state[] achieved_states) {
        rotation.setAngle(pose_rotation.getDegrees()+90.0);
        for(int i = 0; i < num_modules; ++i) {
            final Translation2d trans = module_translations[i];
            final module_state desired = desired_states[i];
            final module_state actual = achieved_states[i];
            desired_module_states[i].setAngle(Rotation2d.fromRadians(desired.theta_rad).minus(trans.getAngle()));
            desired_module_states[i].setLength(desired.speed_mps / max_module_speed);
            actual_module_states[i].setAngle(Rotation2d.fromRadians(actual.theta_rad).minus(trans.getAngle()));
            actual_module_states[i].setLength(actual.speed_mps / max_module_speed);
        }
    }

    private final int num_modules;
    private final Translation2d[] module_translations;
    private final double max_module_speed;
    private Mechanism2d mech;
    private MechanismRoot2d root;
    private MechanismLigament2d rotation;
    private MechanismLigament2d visual_rotation;
    private MechanismLigament2d[] desired_module_states, actual_module_states;
}
