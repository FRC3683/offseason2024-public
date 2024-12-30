package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class sim_ligament2d extends MechanismLigament2d {

    static enum type {
        ELEVATOR,
        ARM,
    }

    private type mtype;
    private MechanismLigament2d curr, target;

    public sim_ligament2d(String name, double length, double angle_offset, double length_offset, type mtype) {
        super(name, length_offset, angle_offset, 0, new Color8Bit());
        this.mtype = mtype;
        curr = append(new MechanismLigament2d(name, length, 0));
        target = append(new MechanismLigament2d(name, length, 0));
    }

    public void update(double curr_val, double target_val) {
        if(mtype == type.ARM) {

        } else {
            
        }
    }

}
