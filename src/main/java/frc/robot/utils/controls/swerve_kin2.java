package frc.robot.utils.controls;

import java.util.Arrays;
import java.util.Collections;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.math_utils;

public class swerve_kin2 extends SwerveDriveKinematics {
    public swerve_kin2(Translation2d[] module_mount_positions, double max_module_mps) {
        super(module_mount_positions);
        chassis_radius = Collections.max(Arrays.asList(module_mount_positions), (pos1, pos2) -> {
            return Double.compare(pos1.getNorm(), pos2.getNorm());
        }).getNorm();
        this.num_modules = module_mount_positions.length;
        this.module_mount_positions = module_mount_positions;
        this.max_module_mps = max_module_mps;
        this.max_chassis_radps = max_module_mps / chassis_radius;
        center_matrices = new swerve_kin2_matrices(new Translation2d(0, 0));
        forward_kin = center_matrices.inverse_kin.pseudoInverse();
        cached_module_states = new module_state[num_modules];
        for(int i = 0; i < num_modules; ++i) {
            cached_module_states[i] = new module_state(0, 0, 0);
        }
    }

    public class swerve_kin2_matrices {
        public swerve_kin2_matrices(Translation2d cot_meters) {
            inverse_kin = new SimpleMatrix(num_modules * 2, 3);
            big_inverse_kin = new SimpleMatrix(num_modules * 2, 4);
            for (int i = 0; i < num_modules; i++) {
                inverse_kin.setRow(
                    i * 2, 0, /* Start Data */ 1, 0, -module_mount_positions[i].getY() + cot_meters.getY());
                inverse_kin.setRow(
                    i * 2 + 1,
                    0, /* Start Data */
                    0,
                    1,
                    +module_mount_positions[i].getX() - cot_meters.getX());
                big_inverse_kin.setRow(
                    i * 2,
                    0, /* Start Data */
                    1,
                    0,
                    -module_mount_positions[i].getX() + cot_meters.getX(),
                    -module_mount_positions[i].getY() + cot_meters.getY());
                big_inverse_kin.setRow(
                    i * 2 + 1,
                    0, /* Start Data */
                    0,
                    1,
                    -module_mount_positions[i].getY() + cot_meters.getY(),
                    +module_mount_positions[i].getX() - cot_meters.getX());
            }
        }

        private final SimpleMatrix inverse_kin;
        private final SimpleMatrix big_inverse_kin;
    }

    public static class module_state implements Comparable<module_state> {
        public double speed_mps;
        public double theta_rad;
        public double omega_radps;

        public module_state(double speed_mps, double theta_rad, double omega_radps) {
            this.speed_mps = speed_mps;
            this.theta_rad = theta_rad;
            this.omega_radps = omega_radps;
        }

        @Override
        public boolean equals(Object obj) {
            if(obj instanceof module_state) {
                module_state other = (module_state) obj;
                return Math.abs(speed_mps - other.speed_mps) <= 0.001
                    && Math.abs(theta_rad - other.theta_rad) <= Units.degreesToRadians(0.1)
                    && Math.abs(omega_radps - other.omega_radps) <= Units.degreesToRadians(0.1);
            }
            return false;
        }

        @Override
        public int compareTo(module_state other) {
            return Double.compare(speed_mps, other.speed_mps);
        }

        public module_state optimize(Rotation2d current_position) {
            var simple_state = new SwerveModuleState(speed_mps, Rotation2d.fromRadians(theta_rad));
            simple_state = SwerveModuleState.optimize(simple_state, current_position);
            speed_mps = simple_state.speedMetersPerSecond;
            theta_rad = simple_state.angle.getRadians();
            return this;
        }
    }

    public void desaturateWheelSpeeds(module_state[] module_states, ChassisSpeeds target_speeds) {
        double target = 0;
        for(int i = 0; i < module_states.length; ++i) {
            target = Math.max(target, Math.abs(module_states[i].speed_mps));
        }
        if(target == 0) {
            return;
        }

        if(target > max_module_mps) {
            for(var state : module_states) {
                state.speed_mps *= (max_module_mps / target);
            }
        }
    }

    /**
     * clamp the input chassis speeds to the given max strafe and max turn speeds
     * turn bias 0.5 is default, clamp strafing and turning equally
     * turn bias 0 is do no turning while strafing at max speed
     * turn bias 1 is do no strafing while turning at max speed
     */
    public ChassisSpeeds clamp(ChassisSpeeds speeds, double turn_bias) {
        // ensure valid bias
        turn_bias = math_utils.clamp01(turn_bias);

        // first clamp strafing and turning individually, so we have each achievable on their own.
        double len = math_utils.hypot(speeds);
        if(len > max_module_mps) {
            double k = max_module_mps / len;
            speeds.vxMetersPerSecond *= k;
            speeds.vyMetersPerSecond *= k;
        }
        speeds.omegaRadiansPerSecond = math_utils.clamp(-max_chassis_radps, max_chassis_radps, speeds.omegaRadiansPerSecond);

        // fun stuff starts here
        double strafe_k = Math.abs(math_utils.hypot(speeds) / max_module_mps); // 0...1
        double turn_k = Math.abs(speeds.omegaRadiansPerSecond / max_chassis_radps); // 0...1
        double k = (strafe_k + turn_k); // 0...2
        final double max_k = max_module_mps / max_module_mps;
        if(k > max_k) { // saturating speeds, need to compensate

            // limit strafing and turning differently based on bias.
            double max_strafe_k = max_k - (turn_k * turn_bias);
            double adjusted_strafe_k = strafe_k * max_strafe_k;
            double adjusted_turn_k = max_k - adjusted_strafe_k;

            speeds.vxMetersPerSecond *= adjusted_strafe_k / strafe_k;
            speeds.vyMetersPerSecond *= adjusted_strafe_k / strafe_k;
            speeds.omegaRadiansPerSecond *= adjusted_turn_k / turn_k;
        }

        return speeds;
    }

    public module_state[] to_module_states(ChassisSpeeds chassis_speeds, swerve_kin2_matrices matrices) {
        if(chassis_speeds.vxMetersPerSecond == 0.0
                && chassis_speeds.vyMetersPerSecond == 0.0
                && chassis_speeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < num_modules; i++) {
                cached_module_states[i].speed_mps = 0.0;
            }
            return cached_module_states;
        }

        var chassis_speeds_vec = new SimpleMatrix(3, 1);
        chassis_speeds_vec.setColumn(
                0,
                0,
                chassis_speeds.vxMetersPerSecond,
                chassis_speeds.vyMetersPerSecond,
                chassis_speeds.omegaRadiansPerSecond);

        var module_velocity_mat = matrices.inverse_kin.mult(chassis_speeds_vec);

        var acceleration_vec = new SimpleMatrix(4, 1);
        acceleration_vec.setColumn(0, 0, 0, 0, Math.pow(chassis_speeds.omegaRadiansPerSecond, 2), 0);

        var module_accel_states_mat = matrices.big_inverse_kin.mult(acceleration_vec);

        for (int i = 0; i < num_modules; i++) {
            double x = module_velocity_mat.get(i * 2, 0);
            double y = module_velocity_mat.get(i * 2 + 1, 0);

            double ax = module_accel_states_mat.get(i * 2, 0);
            double ay = module_accel_states_mat.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            var trig_theta_angle_mat = new SimpleMatrix(2, 2);
            trig_theta_angle_mat.setColumn(0, 0, angle.getCos(), -angle.getSin());
            trig_theta_angle_mat.setColumn(1, 0, angle.getSin(), angle.getCos());

            var accel_vec = new SimpleMatrix(2, 1);
            accel_vec.setColumn(0, 0, ax, ay);

            var omegaVector = trig_theta_angle_mat.mult(accel_vec);

            double omega = (omegaVector.get(1, 0) / speed) - chassis_speeds.omegaRadiansPerSecond;
            cached_module_states[i].speed_mps = speed;
            cached_module_states[i].theta_rad = angle.getRadians();
            cached_module_states[i].omega_radps = omega;
        }

        return cached_module_states;
    }

    public static boolean is_stopped(module_state[] states) {
        for(var state : states) {
            if(Math.abs(state.speed_mps) > 0.005) {
                return false;
            }
        }
        return true;
    }

    public module_state[] form_x() {
        var states = new module_state[num_modules];
        for(int i = 0; i < states.length; ++i) {
            states[i] = new module_state(0, module_mount_positions[i].getAngle().getRadians(), 0);
        }
        return states;
    }

    public static boolean is_strafing(ChassisSpeeds speeds) {
        return !math_utils.slower_than(speeds, 0.01);
    }

    public static boolean is_turning(ChassisSpeeds speeds) {
        return Math.abs(speeds.omegaRadiansPerSecond) >= Units.degreesToRadians(0.1);
    }

    public static boolean is_moving(ChassisSpeeds chassis_speeds) {
        return is_strafing(chassis_speeds) || is_turning(chassis_speeds);
    }

    public module_state[] to_module_states(ChassisSpeeds chassis_speeds) {
        return to_module_states(chassis_speeds, center_matrices);
    }

    public ChassisSpeeds to_chassis_speeds(module_state... module_states) {
        if (module_states.length != num_modules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var module_states_mat = new SimpleMatrix(num_modules * 2, 1);

        for (int i = 0; i < num_modules; i++) {
            var module_state = module_states[i];
            module_states_mat.set(i * 2, 0, module_state.speed_mps * Math.cos(module_state.theta_rad));
            module_states_mat.set(i * 2 + 1, module_state.speed_mps * Math.sin(module_state.theta_rad));
        }

        var chassisSpeedsVector = forward_kin.mult(module_states_mat);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }


    public final int num_modules;
    public final Translation2d[] module_mount_positions;
    public final double chassis_radius;

    public final double max_module_mps;
    public final double max_chassis_radps;

    private final swerve_kin2_matrices center_matrices;
    private final SimpleMatrix forward_kin;

    private module_state[] cached_module_states;
}
