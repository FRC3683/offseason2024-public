// modified WPILIB SlewRateLimiter class to play nice with chassis speeds

package frc.robot.utils.controls;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.asym_slew;
import frc.robot.utils.math_utils;

/**
 * orbit thing w/ different accel and decel, based on dot between current vel and target vel
 **/
public class swerve_slew2 {

    private final double strafe_accel_limit; // max change per calculate call
    private final double strafe_decel_limit; // max change per calculate call
    private final double omega_accel_limit; // max change per calculate call
    private final double omega_decel_limit; // max change per calculate call

    public swerve_slew2(double strafe_accel_limit_s, double strafe_decel_limit_s, double omega_accel_limit_s, double omega_decel_limit_s, double control_dts) {
        strafe_accel_limit = strafe_accel_limit_s * control_dts;
        strafe_decel_limit = strafe_decel_limit_s * control_dts;
        omega_accel_limit = omega_accel_limit_s * control_dts;
        omega_decel_limit = omega_decel_limit_s * control_dts;
    }

    public void calculate(ChassisSpeeds input, ChassisSpeeds prev) {
        double strafe_rate_limit = 0;
        if(!swerve_kin2.is_strafing(input)) {
            strafe_rate_limit = strafe_decel_limit;
        }
        else if(!swerve_kin2.is_strafing(prev)) {
            strafe_rate_limit = strafe_accel_limit;
        } else {
            final double k = math_utils.dot(
                prev.vxMetersPerSecond, prev.vyMetersPerSecond,
                input.vxMetersPerSecond, input.vyMetersPerSecond
            ) / math_utils.hypot(prev) / math_utils.hypot(input);
            SmartDashboard.putNumber("k", k);
            strafe_rate_limit = math_utils.remap(-1, 1, k, strafe_decel_limit, strafe_accel_limit);
        }
        final double omega_rate_limit = asym_slew.growing(input.omegaRadiansPerSecond, prev.omegaRadiansPerSecond) ? omega_accel_limit : omega_decel_limit;

        input.vxMetersPerSecond = math_utils.move_toward(prev.vxMetersPerSecond, input.vxMetersPerSecond, strafe_rate_limit);
        input.vyMetersPerSecond = math_utils.move_toward(prev.vyMetersPerSecond, input.vyMetersPerSecond, strafe_rate_limit);
        input.omegaRadiansPerSecond = math_utils.move_toward(prev.omegaRadiansPerSecond, input.omegaRadiansPerSecond, omega_rate_limit);
    }
}
