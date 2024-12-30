package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class asym_slew {

    private double grow_rate_limit, shrink_rate_limit;
    private double prev_val, prev_time;

    /**
     * rather than positive and negative rate limits, limit moving away from and towards 0 at different rates. 
     * Useful for accel/decel slew on drivetrain.
     * 
     * @param grow_rate_limit expected to be positive
     * @param shrink_rate_limit expected to be positive
     * @param initial_value
     */
    public asym_slew(double grow_rate_limit, double shrink_rate_limit, double initial_value) {
        set_slew(grow_rate_limit, shrink_rate_limit);
        reset(initial_value);
    }

    public void set_slew(double grow_rate_limit, double shrink_rate_limit) {
        this.grow_rate_limit = grow_rate_limit;
        this.shrink_rate_limit = shrink_rate_limit;
    }

    public void reset(double value) {
        prev_val = value;
        prev_time = MathSharedStore.getTimestamp();
    }

    public static boolean growing(double input, double prev) {
        return (Math.signum(prev) == 0) || (Math.signum(input - prev) == Math.signum(prev));
    }

    public double calculate(double input) {
        double current_time = MathSharedStore.getTimestamp();
        double elapsedTime = current_time - prev_time;
        prev_time = current_time;
        boolean growing = growing(input, prev_val);
        double limit = growing ? grow_rate_limit : shrink_rate_limit;
        prev_val +=
            MathUtil.clamp(
                input - prev_val,
                -limit * elapsedTime,
                limit * elapsedTime);
        return prev_val;
    }
}
