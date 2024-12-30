package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

public final class LL_connected_check {

    private final String nt_name;
    private final Debouncer hb_debounce = new Debouncer(0.1, DebounceType.kFalling);
    private double prev_heartbeat = 0;

    public LL_connected_check(String nt_name) {
        this.nt_name = nt_name;
    }

    public boolean connected() {
        double curr_heartbeat = LimelightHelpers.getLimelightNTDouble(nt_name, "hb");
        boolean ok = hb_debounce.calculate(curr_heartbeat != prev_heartbeat);
        prev_heartbeat = curr_heartbeat;
        SmartDashboard.putBoolean("has"+nt_name, ok);
        return ok;
    }
}
