package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

public class clamped_pid extends PIDController {

    private double max_output;
    private double deadzone;

    public clamped_pid(double kp, double ki, double kd, double max_output, double deadzone) {
        super(kp, ki, kd);
        this.max_output = max_output;
        this.deadzone = deadzone;
    }

    public clamped_pid(double kp, double ki, double kd, double max_output, double deadzone, double period) {
        super(kp, ki, kd, period);
        this.max_output = max_output;
        this.deadzone = deadzone;
    }

    public boolean within(double tol) {
        return Math.abs(getPositionError()) <= tol;
    }

    @Override
    public double calculate(double measurement) {
        double val = math_utils.clamp(-max_output, max_output, super.calculate(measurement));
        if(within(deadzone)) {
            return 0;
        }
        return val;
    }
}
