package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.utils.dummy;
import frc.robot.utils.field_util;
import frc.robot.utils.math_utils;
import frc.robot.utils.clamped_pid;
import frc.robot.utils.dave_auto;
import frc.robot.utils.controls.swerve_lowlevel;
import frc.robot.utils.controls.swerve_slew3;

import static frc.robot.constants.swerve.*;

public class swerve extends swerve_lowlevel implements dave_auto.swerve {

    private ChassisSpeeds desired_field_relative_speeds = new ChassisSpeeds();
    private final swerve_slew3 slew_tele = new swerve_slew3(slew_strafe_accel, slew_omega, control_dts);
    private final swerve_slew3 slew_auto = new swerve_slew3(slew_strafe_accel*0.7, slew_omega, control_dts);
    protected boolean sees_note_debounced = false;
    protected Debouncer note_debounce = new Debouncer(0.3, DebounceType.kFalling);
    protected Translation2d note_pose = new Translation2d();

    protected boolean sees_speaker_debounced = false;
    protected Debouncer speaker_debounce = new Debouncer(1.0, DebounceType.kFalling);

    public final dummy strafe_subsystem, omega_subsystem;
    // IMPORTANT: dts is command dts, not swerve dts. Controller sets speeds in commands, wich run at default_period.
    protected final clamped_pid theta_con_rad = new clamped_pid(theta_kP, theta_kI, theta_kD, max_snap_radps, theta_con_deadzone, constants.control_dts);
    protected final clamped_pid x_con_m = new clamped_pid(strafe_kP, strafe_kI, strafe_kD, max_speed_mps, 0, constants.control_dts);
    protected final clamped_pid y_con_m = new clamped_pid(strafe_kP, strafe_kI, strafe_kD, max_speed_mps, 0, constants.control_dts);

    private final String turning_bias_pref_key = "swerve_turning_bias";

    public swerve(robot robot) {
        super(robot);
        strafe_subsystem = new dummy();
        omega_subsystem = new dummy();

        theta_con_rad.enableContinuousInput(-Math.PI, Math.PI);
        theta_con_rad.setTolerance(Units.degreesToRadians(1));

        Preferences.initDouble(turning_bias_pref_key, 0.6);
    }

    @Override
    public ChassisSpeeds calc_desired_field_relative_speeds() {
        // probably want a different turn bias for auto and teleop, and driver preference
        kin.clamp(desired_field_relative_speeds, Preferences.getDouble(turning_bias_pref_key, 0.6));
        var slewed = desired_field_relative_speeds.times(1); // copy speeds
        var field_speeds = get_field_relative_speeds();
        if(DriverStation.isAutonomous()) {
            slew_auto.calculate(slewed, field_speeds);
        } else {
            slew_tele.calculate(slewed, field_speeds);
        }
        return slewed;
    }

    public boolean sees_note() {
        return sees_note_debounced;
    }

    public double note_dist() {
        return note_pose.minus(get_pose().getTranslation()).getNorm();
    }

    public boolean sees_speaker() {
        return sees_speaker_debounced;
    }

    public double speaker_dist() {
        return field_util.fix(constants.field.speaker_pos_blue).minus(get_pose().getTranslation()).getNorm();
    }

    protected void update_note_pose(Pose2d fused_pose) {
        var sees_note_rn = LimelightHelpers.getTV(config.LL_intake);
        if(sees_note_rn && Math.abs(heading_rate_signal.getValue()) < 100) {
            var tl = LimelightHelpers.getLatency_Pipeline(config.LL_intake);
            var tc = LimelightHelpers.getLatency_Capture(config.LL_intake);
            final var other_latency = 11; // 
            var total_latency = (tl + tc + other_latency) / 1000.0; // seconds

            var speeds = get_speeds().times(total_latency);

            var tx = Math.toRadians(LimelightHelpers.getTX(config.LL_intake));
            var ty = Math.toRadians(LimelightHelpers.getTY(config.LL_intake) - 22.5);
            var h = config.intake_mount.mount_offset.getZ();
            var x = h * Math.tan(ty + config.intake_mount.mount_angle.getRadians()) - speeds.vxMetersPerSecond;
            var y = - Math.tan(tx) * x - speeds.vyMetersPerSecond;
            var ll_to_note = new Translation2d(x, y);
            var center_to_ll = config.intake_mount.mount_offset.toTranslation2d();

            var center_to_note = (center_to_ll.plus(ll_to_note)).rotateBy(
                get_heading().minus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond)));

            note_pose = fused_pose.getTranslation().plus(center_to_note);

            field.getObject("note").setPose(new Pose2d(note_pose, new Rotation2d()));
        }
        sees_note_debounced = note_debounce.calculate(sees_note_rn);
    }

    public Translation2d calc_relative_note_offset() {
        var diff = note_pose.minus(get_pose().getTranslation()).rotateBy(get_heading().times(-1));
        SmartDashboard.putString("note relative", diff.toString());
        return diff;
    }

    public ChassisSpeeds calc_intake_assist(ChassisSpeeds speeds) {
        ChassisSpeeds assist = new ChassisSpeeds();
        final double assist_kP = 3.8;
        if(sees_note() && speeds.vxMetersPerSecond > 0.05) {
            var note_offset = calc_relative_note_offset();
            assist.vyMetersPerSecond = math_utils.clamp(-3.5, 3.5, note_offset.getY() * assist_kP);
        }
        return assist;
    }

    public boolean theta_within(double tol_deg) {
        return theta_con_rad.within(Units.degreesToRadians(tol_deg));
    }

    public Command cmd_reset_pose(Pose2d pose) {
        return Commands.runOnce(() -> {
            reset_pose(pose);
        }).ignoringDisable(true);
    }

    public Command cmd_reject_vision(boolean do_reject) {
        return Commands.runOnce(() -> {
            reject_vision_pose(do_reject);
        });
    }

    @Override
    public void reset_pose(Pose2d pose) {
        super.reset_pose(pose);
    }

    public Command make_form_x_command() {
        ChassisSpeeds stop = new ChassisSpeeds(0, 0, 0);
        return baselock_when_stop(strafe_field_relative(() -> stop).alongWith(turn(() -> 0.0)));
    }

    public Command baselock_when_stop(Command command) {
        return Commands.runOnce(() -> {
            form_x_when_stopped = true;
        })
        .andThen(command)
        .finallyDo(() -> {
            form_x_when_stopped = false;
        });
    }

    public Command baselock_when_stop() {
        return baselock_when_stop(Commands.idle());
    }

    public void set_strafe(ChassisSpeeds speeds) {
        desired_field_relative_speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
        desired_field_relative_speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    }

    public void set_turn(double omega_radps) {
        desired_field_relative_speeds.omegaRadiansPerSecond = omega_radps;
    }

    public void set_swerve(ChassisSpeeds speeds) {
        set_strafe(speeds);
        set_turn(speeds.omegaRadiansPerSecond);
    }

    public Command sysid(double initial_voltage, double delta_v) {
        var speeds = new ChassisSpeeds(initial_voltage / 12.0 * max_module_speed_mps, 0, 0);
        return Commands.runOnce(() -> {
            optimize_azimuth = false;
        }).andThen(Commands.runEnd(() -> {
            set_swerve(speeds);
            speeds.vxMetersPerSecond += delta_v / 12.0 * max_module_speed_mps;
        }, () -> {
            optimize_azimuth = true;
        }, strafe_subsystem, omega_subsystem));
    }

    public Command sysid_quasistatic(double delta_v) {
        return sysid(0, delta_v);
    }

    public Command sysid_dynamic(double step_voltage) {
        return sysid(step_voltage, 0);
    }

    public Command strafe_field_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return Commands.run(() -> {
            set_strafe(strafe_supplier.get());
        }, strafe_subsystem)
        .finallyDo(() -> {
            set_strafe(new ChassisSpeeds(0, 0, 0));
        });
    }

    public Command strafe_robot_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return strafe_field_relative(() -> ChassisSpeeds.fromRobotRelativeSpeeds(strafe_supplier.get(), get_heading()));
    }

    public Command strafe_omega(Supplier<ChassisSpeeds> supplier) {
        return strafe_field_relative(supplier).alongWith(turn(() -> supplier.get().omegaRadiansPerSecond));
    }

    @Override
    public Command strafe_to_point(final Translation2d point, final double max_vel, final double tolerance, final boolean through) {
        return Commands.runOnce(() -> {
            x_con_m.setTolerance(tolerance);
            x_con_m.reset();
        })
        .andThen(strafe_field_relative(() -> {
            var err = point.minus(get_pose().getTranslation());
            double err_len = err.getNorm();
            double speed = through ? max_vel
                                : math_utils.clamp(-max_vel, max_vel, x_con_m.calculate(-err_len, 0));
            var output = err.div(err_len == 0 ? 1 : err_len).times(speed);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        }).until(() -> point.getDistance(get_pose().getTranslation()) <= tolerance));
    }
    public Command strafe_to_fixed_point(Translation2d point, final double tolerance, final boolean through) {
        return strafe_to_point(field_util.fix(point), max_speed_mps, tolerance, through);
    }
    public Command strafe_to_fixed_point(Translation2d point, final double max_vel, final double tolerance, final boolean through) {
        return strafe_to_point(field_util.fix(point), max_vel, tolerance, through);
    }

    @Override
    public Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel) {
        return strafe_field_relative(() -> {
            var err = point.minus(get_pose().getTranslation()).rotateBy(direction.unaryMinus());
            var y_out = math_utils.clamp(-max_vel*pid_line_y_weight, max_vel*pid_line_y_weight, y_con_m.calculate(-err.getY(), 0));
            var x_lim = Math.sqrt( math_utils.sq(max_vel) - math_utils.sq(y_out) );
            var x_out = math_utils.clamp(-x_lim, x_lim, x_con_m.calculate(-err.getX(), 0) + through_vel);
            var output = new Translation2d(
                x_out,
                y_out
            ).rotateBy(direction);
            output = math_utils.clamp(output, max_vel);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        }).until(() -> math_utils.close_enough(point, get_pose().getTranslation(), tolerance));
    }

    public Command strafe_line(Translation2d point, Rotation2d direction, double tolerance, final double through_vel) {
        return strafe_line(point, direction, max_speed_mps, tolerance, through_vel);
    }

    public Command strafe_line_fixed(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel) {
        return strafe_line(field_util.fix(point), field_util.fix(direction), max_vel, tolerance, through_vel);
    }

    public Command strafe_line_fixed(Translation2d point, Rotation2d direction, double tolerance, final double through_vel) {
        return strafe_line(field_util.fix(point), field_util.fix(direction), tolerance, through_vel);
    }

    public Command strafe_arc(Translation2d point, Translation2d arc_center, boolean clockwise, double max_vel, double tolerance) {
        final var arc_radius = point.minus(arc_center).getNorm();
        final var target_angle = point.minus(arc_center).getAngle();
        return strafe_field_relative(() -> {
            var pose = get_pose().getTranslation();
            var curr_dist = pose.minus(arc_center);
            var curr_angle = curr_dist.getAngle();
            var target_dist = new Translation2d(arc_radius, curr_angle);
            var angle_err = math_utils.err(target_angle, curr_angle, clockwise);
            var dist_err = target_dist.minus(curr_dist);
            var x_lim = math_utils.remap_clamp(0.2, 0.9, Math.abs(dist_err.getNorm()), max_vel, 0);
            var err_arc = angle_err.getRadians() * curr_dist.getNorm();
            var arc_speed = math_utils.clamp(-x_lim, x_lim, x_con_m.calculate(-err_arc, 0));
            var dist_speed = math_utils.clamp(-max_vel, max_vel, y_con_m.calculate(-dist_err.getNorm(), 0));
            var arc_output = curr_dist.div(curr_dist.getNorm()).rotateBy(Rotation2d.fromDegrees(clockwise ? -90 : 90)).times(arc_speed);
            var center_output = curr_dist.div(curr_dist.getNorm()).times(-dist_speed);
            var output = arc_output.plus(center_output);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        }).until(() -> math_utils.close_enough(point, get_pose().getTranslation(), tolerance));
    }

    public Command strafe_arc_fixed(Translation2d point, Translation2d arc_center, boolean clockwise, double max_vel, double tolerance) {
        return strafe_arc(field_util.fix(point), field_util.fix(arc_center), clockwise, max_vel, tolerance);
    }

    @Override
    public Command strafe_assisted(Translation2d target, Rotation2d direction, double max_vel, double tolerance, final boolean through) {
        return Commands.runOnce(() -> {
            x_con_m.setTolerance(tolerance);
        })
        .andThen(strafe_field_relative(() -> {
            var err = target.minus(get_pose().getTranslation()).rotateBy(direction.unaryMinus());
            double speed = math_utils.clamp(-max_vel, max_vel, x_con_m.calculate(-err.getX(), 0));
            var output = new Translation2d(speed, 0).rotateBy(direction);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
                //.plus(calc_intake_assist(get_speeds()));
        }).until(() -> x_con_m.atSetpoint()));
    }
    public Command strafe_assisted(Translation2d target, Rotation2d direction, double tolerance, final boolean through) {
        return strafe_assisted(target, direction, 2.0, tolerance, through);
    }
    public Command strafe_assisted_fixed(Translation2d target, Rotation2d direction, double tolerance, final boolean through) {
        return strafe_assisted(field_util.fix(target), field_util.fix(direction), tolerance, through);
    }

    public Command turn(Supplier<Double> omega_rad_supplier) {
        return Commands.run(() -> {
            set_turn(omega_rad_supplier.get());
        }, omega_subsystem)
        .finallyDo(() -> {
            set_turn(0.0);
        });
    }

    public Command maintain_heading() {
        var wrapper = new Object() { Rotation2d angle; };
        return Commands.sequence(
            Commands.runOnce(() -> {
                wrapper.angle = get_heading();
            }),
            snap(() -> wrapper.angle.getDegrees())
        );
    }

    @Override
    public Command snap(double theta_deg) {
        return snap_with_omega(() -> new theta_omega(Units.degreesToRadians(theta_deg), 0.0));
    }
    public Command fixed_snap(double theta_deg) {
        return snap(field_util.fix(Rotation2d.fromDegrees(theta_deg)).getDegrees());
    }
    public Command snap(Supplier<Double> theta_deg) {
        return snap_with_omega(() -> new theta_omega(Units.degreesToRadians(theta_deg.get()), 0));
    }

    class theta_omega {
        double theta_rad, omega_radps;
        theta_omega(double theta_deg, double omega_degps) {
            this.theta_rad = theta_deg;
            this.omega_radps = omega_degps;
        }
    }

    public Command snap_with_omega(Supplier<theta_omega> supplier) {
        return turn(() -> {
            var thetas = supplier.get();
            var feedback = theta_con_rad.calculate(get_heading().getRadians(), thetas.theta_rad);
            var feedforward = thetas.omega_radps;
            return feedback + feedforward;
        });
    }

    public Command snap_trans(Supplier<Translation2d> look_at, double offset_deg) {
        return snap_with_omega(() -> {
            var pose = get_pose().getTranslation();
            var target = look_at.get();
            var diff = target.minus(pose);
            double target_rad = diff.getAngle().getRadians() + Units.degreesToRadians(offset_deg);
            var speeds = math_utils.trans(get_field_relative_speeds()).times(control_dts);
            var next_pose = pose.plus(speeds);
            var next_diff = target.minus(next_pose);
            var next_target = next_diff.getAngle().getRadians() + Units.degreesToRadians(offset_deg);
            var ang_vel = (next_target - target_rad) / control_dts;
            var vel_kP = 1;
            return new theta_omega(target_rad, ang_vel * vel_kP);
        });
    }
    public Command snap_trans(Translation2d look_at, double offset_deg) {
        return snap_trans(() -> look_at, offset_deg);
    }

    public Command stop_motors() {
        return Commands.startEnd(() -> {
            stop_motors = true;
        }, () -> {
            stop_motors = false;
        }, strafe_subsystem, omega_subsystem);
    }

}
