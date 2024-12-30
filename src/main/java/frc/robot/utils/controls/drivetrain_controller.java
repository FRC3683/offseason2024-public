package frc.robot.utils.controls;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.math_utils;

/**
 * for when a mechanism (usually a drivetrain) already has PID running on the motors, so we close loop position with a linear
 * deceleration profile instead of another P loop on top, which would have non-optimal polynomial deceleration
 * 
 * we still use a P controller within a certain threshold, to avoid oscillation caused by discrete time-steps
 * unfortunately, this threshold has to be quite big on the swerve theta controller (30 degrees?!?!), cest la vie.
 * 
 * we automatically compute an optimal kP based on the target intermediate velocity and p_threshold, to ensure a continuous 
 * velocity over distance function, ie. smooth deceleration over the p_threshold
 */
public class drivetrain_controller {
    
    private final double max_vel, max_accel, epsilon, p_threshold;
    private double tolerance, kP, prev_target_vel;
    private double err;
    private boolean is_continuous;
    private double min_continuous_input, max_continuous_input;

    public drivetrain_controller(double max_vel, double max_accel, double p_threshold, double control_dts) {
        this.max_vel = max_vel;
        this.epsilon = max_vel * 0.3 * control_dts;
        this.max_accel = max_accel;
        this.p_threshold = p_threshold;
        update_kp(0);
    }

    private void update_kp(double target_vel) {
        prev_target_vel = target_vel;
        kP = Math.abs(calc_ideal(p_threshold, target_vel) - target_vel) / p_threshold; // magic
    }

    public void enable_continuous(double min, double max) {
        is_continuous = true;
        min_continuous_input = min;
        max_continuous_input = max;
    }

    public void disable_continuous() {
        is_continuous = false;
    }

    public boolean atSetpoint() {
        return within(tolerance);
    }

    public boolean within(double tolerance) {
        return MathUtil.isNear(0, err, tolerance);
    }

    // the main math that would work on its own if we weren't limited to such big time steps (even 250hz doesnt seem like enough in sim)
    private double calc_ideal(double err, double target_vel) {
        if(err > 0) {
            err -= epsilon;
        } else {
            err += epsilon;
        }
        return math_utils.clamp(-max_vel, max_vel, Math.signum(err) * Math.sqrt(math_utils.sq(target_vel) + Math.abs(2 * max_accel * err))); // cool af
    }

    public double calculate(double target_pos, double current_pos, double target_vel) {
        target_vel = math_utils.clamp(-max_vel, max_vel, target_vel);

        if(is_continuous) {
            double error_bound = (max_continuous_input - min_continuous_input) / 2.0;
            err = MathUtil.inputModulus(target_pos - current_pos, -error_bound, error_bound);
        } else {
            err = target_pos - current_pos;
        }

        // discrete time-steps cause oscillation without this
        if(Math.abs(err) < p_threshold) {
            if(target_vel != prev_target_vel) {
                update_kp(target_vel); // update kP for different intermediate velocities
            }
            return math_utils.clamp(-max_vel, max_vel, (kP * err) + target_vel);
        }

        // vf^2 = vi^2 + 2ad
        // vi = sqrt(vf^2 - 2ad) * -signum(err)
        // vf = target_vel, vi = output, a = max_accel, d = err
        return calc_ideal(err, target_vel);
    }

    public double calculate(double error, double target_vel) {
        return calculate(0, error, target_vel);
    }

    public double calculate(double error) {
        return calculate(error, 0);
    }

    public void setTolerance(double tol) {
        tolerance = tol;
    }

    public double get_tolerance() {
        return tolerance;
    }
}
