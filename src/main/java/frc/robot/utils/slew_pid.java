package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

public class slew_pid {
    private final PIDController con;
    private final asym_slew slew;
    private double max_vel;
    private boolean zero_at_target = false;

    public slew_pid(double kP, double kI, double kD, double max_vel, double max_accel) {
        con = new PIDController(kP, kI, kD);
        this.slew = new asym_slew(max_accel, 100000, 0);
        this.max_vel = max_vel;
    }

    public void enable_zero_at_target() {
        zero_at_target = true;
    }

    public double get_position_error() {
        return con.getPositionError();
    }

    public void set_constraints(double max_vel, double max_accel) {
        slew.set_slew(max_accel, 100000);
        this.max_vel = max_vel;
    }

    public void enable_continuous_input(double min, double max) {
        con.enableContinuousInput(min, max);
    }

    public void set_tolerance(double tol) {
        con.setTolerance(tol);
    }

    public double get_tolerance() {
        return con.getPositionTolerance();
    }

    public boolean at_goal() {
        return con.atSetpoint();
    }


    public double calculate(double input, double target) {
        double con_out = con.calculate(input, target);
        double raw = math_utils.clamp(-max_vel, max_vel, con_out);
        if(zero_at_target && Math.abs(con.getPositionError()) < con.getPositionTolerance() / 2.0) {
            raw = 0;
        }
        var slewed = slew.calculate(raw);
        return slewed;
    }

    public double calculate(double input) {
        return calculate(input, 0);
    }

    public void reset(double vel) {
        con.reset();
        slew.reset(vel);
    }
}
