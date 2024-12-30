package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.field_util;

public class constants {
    public static final int control_freq = 50;
    public static final double control_dts = 1.0 / control_freq;

    public static final double voltage_warning_threshold_comp = 12.6;
    public static final double voltage_warning_threshold_prac = 12.1;

    public final class field {

        public static final Translation2d speaker_pos_blue = new Translation2d(0.05, 5.56);
        public static final Translation2d amp_pos = new Translation2d();
        public static final Translation2d lob_pos = new Translation2d(1.0, 7.0);
        public static final Translation2d blue_stage = new Translation2d(4.93, field_util.field_size_m.getY() / 2.0);
        public static final Translation2d red_stage = field_util.flip(blue_stage);
        // public static final double lob_line_blue_m = 7.7;
        // public static final double lob_line_red_m = field_util.field_size_m.getX() - lob_line_blue_m;
        public static final double wingline_blue_m = 6;
        public static final double wingline_red_m = field_util.field_size_m.getX() - wingline_blue_m;
        public static final double centerline = field_util.field_size_m.getX() / 2.0;

        public static final double[] stage_normals = {
            Math.toRadians(120), // blue left
            Math.toRadians(240), // blue right
            0,                          // blue center
            Math.PI,                    // red center
            Math.toRadians(300), // red left
            Math.toRadians(60)   // red right
        };

        public static int stage_index(Translation2d position) {
            var blue = position.getX() < centerline;
            var stage = blue ? blue_stage : red_stage;
            var angle = Math.toDegrees(MathUtil.angleModulus(position.minus(stage).getAngle().getRadians())); // -180 <= angle <= 180
            if(blue) {
                if(angle > 60) {
                    return 0; // blue left
                }
                if(angle < -60) {
                    return 1; // blue right
                }
                return 2; // blue center (far)
            }
            if(angle >= 120 || angle <= -120) {
                return 3; // red center (far)
            }
            if(angle < 0) {
                return 4; // red left
            }
            return 5; // red right
        }

        public static Rotation2d stage_normal(Translation2d position) {
            return Rotation2d.fromRadians(stage_normals[stage_index(position)]);
        }
    }

    public final class turret {
        public static final double offset_mini = 0.37;
        public static final double offset_minier = 0.037;
        public final static int main_gear = 46;
        public final static int mini_gear = 15;
        public final static int minier_gear = 11;
    }

    public final class swerve {
        public enum module_e {
            mk4i_L1(8.14, 150.0/7.0, (25.0 / 19.0) * (15.0 / 45.0)),
            mk4i_L2(6.75, 150.0/7.0, (27.0 / 17.0) * (15.0 / 45.0)),
            mk4i_L3(6.12, 150.0/7.0, (28.0 / 16.0) * (15.0 / 45.0)),
            ;

            public final double drive_ratio, steer_ratio, couple_ratio;
            module_e(double drive_ratio, double steer_ratio, double couple_ratio) {
                this.drive_ratio = drive_ratio;
                this.steer_ratio = steer_ratio;
                this.couple_ratio = couple_ratio;
            }
        }

        private static final int default_odom_freq = 100;
        private static final int canivore_odom_freq = 200;
        public static final int odom_freq = (RobotBase.isSimulation() || config.drive_canbus == config.can_ivore) ? canivore_odom_freq : default_odom_freq;
        public static final double odom_dts = 1.0/odom_freq;
        public static final int control_freq = 100;
        public static final double control_dts = 1.0/control_freq;

        public static final module_e module = module_e.mk4i_L3;
        private static final double k = 1.01634;
        public static final double wheel_diameter = Units.inchesToMeters(3.75) * k;
        public static final double wheel_radius = wheel_diameter / 2.0;
        
        public static final double amp_throttle = 0.4, shot_throttle = 0.4, climb_throttle = 0.6;

        public static final double half_wheelbase_meters = 0.52705 / 2.0;
        public static final double max_module_speed_mps = 4.572;
        public static final double max_speed_mps = 4.3;
        public static final double slew_strafe_accel = max_speed_mps / 0.2; // 0 to 4.3 in 200ms
        public static final double slew_strafe_decel = max_speed_mps / 0.3; // 4.3 to 0 in 300ms
        public static final double max_strafe_decel_auto = slew_strafe_decel * 0.95;
        public static final double slew_omega = 9.0 / 0.15; // 0 to 9 radps in 150ms
        public static final double max_snap_radps = 8.0;
        public static final double max_snap_radps2 = slew_omega * 0.8;
        public static final double pid_line_y_weight = 0.95;

        public static double theta_con_deadzone = Units.degreesToRadians(0.5);
        public static final double theta_kP = 7.0, theta_kI = 0.0, theta_kD = 0.1;
        public static final double strafe_kP = 8.0, strafe_kI = 0.0, strafe_kD = 1.0;

        public static final Translation2d offset_fr = new Translation2d(half_wheelbase_meters, -half_wheelbase_meters);
        public static final Translation2d offset_fl = new Translation2d(half_wheelbase_meters, half_wheelbase_meters);
        public static final Translation2d offset_br = new Translation2d(-half_wheelbase_meters, -half_wheelbase_meters);
        public static final Translation2d offset_bl = new Translation2d(-half_wheelbase_meters, half_wheelbase_meters);

        public static final Translation2d[] module_offsets = { offset_fr, offset_fl, offset_br, offset_bl };

        public static final Matrix<N3, N1> cam1_st_devs = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(999999));
    }
}
