package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * chinese remainder theorem dual encoder turret
 */
public class crt_sim {
    private double turret_pos, enc1_pos, enc2_pos;

    final String nt_entry;
    final double ratio1, ratio2;
    final double max_noise;

    public crt_sim(String nt_entry, int main_gear, int gear1, int gear2, double max_noise_deg) {
        this.nt_entry = nt_entry;
        ratio1 = (double)main_gear / (double)gear1;
        ratio2 = (double)main_gear / (double)gear2;
        max_noise = Units.degreesToRotations(max_noise_deg);
        set(180);
    }

    private double noise() {
        return (Math.random() - 0.5) * max_noise;
    }

    public void set(double turret_pos_deg) {
        turret_pos = Units.degreesToRotations(turret_pos_deg);
        enc1_pos = MathUtil.inputModulus(turret_pos * ratio1 + noise(), 0, 1);
        enc2_pos = MathUtil.inputModulus(turret_pos * ratio2 + noise(), 0, 1);
        SmartDashboard.putNumber(nt_entry+"/main", turret_pos_deg);
        SmartDashboard.putNumber(nt_entry+"/enc1", enc1_pos);
        SmartDashboard.putNumber(nt_entry+"/enc2", enc2_pos);
    }

    public void add(double turret_deg) {
        set(Units.rotationsToDegrees(turret_pos) + turret_deg);
    }

    public double get_main() {
        return turret_pos;
    }

    public double get_enc1() {
        return enc1_pos;
    }

    public double get_enc2() {
        return enc2_pos;
    }
}
