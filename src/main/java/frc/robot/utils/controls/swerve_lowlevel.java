package frc.robot.utils.controls;

import static frc.robot.constants.swerve.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.sim.swerve_mech2d;
import frc.robot.utils.configurable;
import frc.robot.utils.math_utils;
import frc.robot.utils.controls.swerve_kin2.module_state;
import frc.robot.utils.controls.swerve_module.drive_request;

public abstract class swerve_lowlevel extends SubsystemBase implements configurable {

    /**
     * called automatically at the frequency control_dts
     * 
     * @return the chassis speeds to apply
     */
    public abstract ChassisSpeeds calc_desired_field_relative_speeds();

    protected final Pigeon2 pig = new Pigeon2(config.can_pigeon, config.drive_canbus);
    public final swerve_module[] modules = {
        new swerve_module(config.swerve.module_configs[0]),
        new swerve_module(config.swerve.module_configs[1]),
        new swerve_module(config.swerve.module_configs[2]),
        new swerve_module(config.swerve.module_configs[3]),
    };
    protected final StatusSignal<Double> heading_signal, heading_rate_signal;

    protected final swerve_kin2 kin = new swerve_kin2(module_offsets, max_module_speed_mps);
    protected final SwerveDriveOdometry raw_odom;
    private final SwerveModulePosition[] module_positions = new SwerveModulePosition[4];
    protected SwerveDrivePoseEstimator pose_estimator;
    protected Pose2d cam_pose = new Pose2d();
    protected boolean reject_vision_updates = false;
    protected double last_vision_timestamp = 0;
    protected double repeated_vision_measurements = 0;

    private final module_state[] form_x_states = kin.form_x();
    protected boolean form_x_when_stopped = false;
    protected boolean stop_motors = false;
    protected drive_request drive_req = drive_request.PID;
    protected boolean optimize_azimuth = true;

    private ChassisSpeeds commanded_field_relative_speeds = new ChassisSpeeds();
    private module_state[] desired_states = {
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0), 
        new module_state(0, 0, 0),
    };
    private module_state[] current_states = {
        modules[0].get_state(),
        modules[1].get_state(),
        modules[2].get_state(),
        modules[3].get_state(),
    };

    protected final Field2d field = new Field2d();
    private final swerve_mech2d mech2d = new swerve_mech2d(kin, 3);

    public swerve_lowlevel(TimedRobot robot) {
        modules[0].init(robot);
        modules[1].init(robot);
        modules[2].init(robot);
        modules[3].init(robot);
        heading_signal = pig.getYaw();
        heading_rate_signal = pig.getAngularVelocityZWorld();
        BaseStatusSignal.setUpdateFrequencyForAll(odom_freq, heading_signal, heading_rate_signal);

        Pose2d pose = new Pose2d();
        update_module_positions();
        raw_odom = new SwerveDriveOdometry(kin, pig.getRotation2d(), module_positions, pose);
        pose_estimator = new SwerveDrivePoseEstimator(kin, pig.getRotation2d(), module_positions, pose);

        robot.addPeriodic(this::odom_periodic, odom_dts);
        robot.addPeriodic(this::nt_periodic, RobotBase.isSimulation() ? 1.0/100.0 : 1.0/16.0, 1.0/32.0);
        robot.addPeriodic(this::controls_periodic, control_dts);

        mech2d.init();
        SmartDashboard.putData(field);
    }

    private void controls_periodic() {
        if(burning_motors() || stop_motors) {
            modules[0].stop();
            modules[1].stop();
            modules[2].stop();
            modules[3].stop();
            return;
        }

        var heading = get_pose().getRotation();

        var desired_field_relative_speeds = calc_desired_field_relative_speeds();
        var desired_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desired_field_relative_speeds, heading);
        if(swerve_kin2.is_moving(desired_chassis_speeds)) {
            desired_states = kin.to_module_states(desired_chassis_speeds);
            kin.desaturateWheelSpeeds(desired_states, desired_chassis_speeds);
        } else if(form_x_when_stopped) {
            desired_states = form_x_states;
        } else {
            desired_states[0].speed_mps = 0;
            desired_states[1].speed_mps = 0;
            desired_states[2].speed_mps = 0;
            desired_states[3].speed_mps = 0;
        }

        modules[0].set_state(desired_states[0], optimize_azimuth, drive_req);
        modules[1].set_state(desired_states[1], optimize_azimuth, drive_req);
        modules[2].set_state(desired_states[2], optimize_azimuth, drive_req);
        modules[3].set_state(desired_states[3], optimize_azimuth, drive_req);

        // swerve_module.set_state() can modify the desired state with cosine scale and optimize
        // best to put the commanded chassis speeds calculation after the set_state()
        var commanded_chassis_speeds = kin.to_chassis_speeds(desired_states);
        commanded_field_relative_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(commanded_chassis_speeds, heading);
    }

    private void nt_periodic() {
        Pose2d raw_pose = get_raw_pose();
        Pose2d fused_pose = get_pose();

        field.setRobotPose(fused_pose);
        field.getObject("cam_pose").setPose(cam_pose);
        
        SmartDashboard.putBoolean("toasty warning", toasty_motors());
        SmartDashboard.putBoolean("burning motors", burning_motors());

        mech2d.update(raw_pose.getRotation(), desired_states, current_states);

        if(config.dev) {
            var speeds = get_field_relative_speeds();
            SmartDashboard.putNumber("swerve/vx", speeds.vxMetersPerSecond);
            SmartDashboard.putNumber("swerve/vy", speeds.vyMetersPerSecond);
            SmartDashboard.putNumber("swerve/vt", speeds.omegaRadiansPerSecond);
            SmartDashboard.putNumber("swerve/x", fused_pose.getX());
            SmartDashboard.putNumber("swerve/theta", get_heading().getRadians());
        }
    }

    public void odom_periodic() {
        double timeout_seconds = (config.drive_canbus == config.can_ivore && config.swerve.pro) ? odom_dts / 2.0 : 0.0;
        BaseStatusSignal.waitForAll(timeout_seconds, 
            heading_signal, heading_rate_signal,
            modules[0].drive_pos_signal, modules[0].drive_vel_signal, modules[0].turn_pos_signal, modules[0].turn_vel_signal,
            modules[1].drive_pos_signal, modules[1].drive_vel_signal, modules[1].turn_pos_signal, modules[1].turn_vel_signal,
            modules[2].drive_pos_signal, modules[2].drive_vel_signal, modules[2].turn_pos_signal, modules[2].turn_vel_signal,
            modules[3].drive_pos_signal, modules[3].drive_vel_signal, modules[3].turn_pos_signal, modules[3].turn_vel_signal
        );

        update_module_positions();
        update_module_states();

        Rotation2d pig_rotation = Rotation2d.fromDegrees(heading_signal.getValue());
        if(RobotBase.isSimulation()) {
            pig_rotation = raw_odom.getPoseMeters().getRotation().plus(Rotation2d.fromRadians(commanded_field_relative_speeds.omegaRadiansPerSecond * odom_dts));
        }
        var raw_pose = raw_odom.update(pig_rotation, module_positions);
        var fused_pose = pose_estimator.update(pig_rotation, module_positions);

        var heading = get_heading();

        LimelightHelpers.SetRobotOrientation(config.LL_shooter, heading.getDegrees(), 
            heading_rate_signal.getValue(), 0, 0, 0, 0);
        if(!reject_vision_updates && Math.abs(heading_rate_signal.getValue()) < 360 && LimelightHelpers.getTV(config.LL_shooter)) { // official chief post example suggest 720
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.LL_shooter);
            if(math_utils.close_enough(last_vision_timestamp, mt2.timestampSeconds, 0.02)) {
                repeated_vision_measurements++;
                SmartDashboard.putNumber("repeated vision measurements", repeated_vision_measurements);
            }
            cam_pose = mt2.pose;
            pose_estimator.addVisionMeasurement(cam_pose, mt2.timestampSeconds, cam1_st_devs);
            last_vision_timestamp = mt2.timestampSeconds;
        }

        ctre_log_pose("cam_pose", cam_pose);
        ctre_log_pose("raw_pose", raw_pose);
        ctre_log_pose("fused_pose", fused_pose);
    }

    public double get_max_chassis_radps() {
        return kin.max_chassis_radps;
    }

    public double chassis_radius(int module) {
        return kin.module_mount_positions[module].getNorm();
    }

    public void reject_vision_pose(boolean reject) {
        reject_vision_updates = reject;
    }

    public void coast() {
        modules[0].coast();
        modules[1].coast();
        modules[2].coast();
        modules[3].coast();
    }

    public boolean toasty_motors() {
        return modules[0].toasty() 
            || modules[1].toasty() 
            || modules[2].toasty()
            || modules[3].toasty();
    }

    public boolean burning_motors() {
        return modules[0].burning()
            || modules[1].burning()
            || modules[2].burning()
            || modules[3].burning();
    }

    public void reset_encoders() {
        modules[0].reset_turning();
        modules[1].reset_turning();
        modules[2].reset_turning();
        modules[3].reset_turning();
    }

    public boolean has_abs() {
        return modules[0].has_abs()
                && modules[1].has_abs()
                && modules[2].has_abs()
                && modules[3].has_abs();
    }

    private void update_module_positions() {
        module_positions[0] = modules[0].odom();
        module_positions[1] = modules[1].odom();
        module_positions[2] = modules[2].odom();
        module_positions[3] = modules[3].odom();
    }

    private void update_module_states() {
        modules[0].update_state();
        modules[1].update_state();
        modules[2].update_state();
        modules[3].update_state();
    }

    public SwerveModulePosition[] copy_module_positions() {
        final int length = module_positions.length;
        SwerveModulePosition[] copy = new SwerveModulePosition[length];
        for(int i = 0; i < length; ++i) {
            copy[i] = new SwerveModulePosition(module_positions[i].distanceMeters, Rotation2d.fromRadians(module_positions[i].angle.getRadians()));
        }
        return copy;
    }

    public ChassisSpeeds get_commanded_speeds() {
        return commanded_field_relative_speeds;
    }

    public void zero_heading(Rotation2d new_heading) {
        reset_pose(new Pose2d(get_pose().getTranslation(), new_heading));
    }

    public void reset_pose(Pose2d pose) {
        raw_odom.resetPosition(pig.getRotation2d(), module_positions, pose);
        pose_estimator.resetPosition(pig.getRotation2d(), module_positions, pose);
        System.out.println(pose);
    }

    public Pose2d get_raw_pose() {
        return raw_odom.getPoseMeters();
    }

    public Pose2d get_pose() {
        return pose_estimator.getEstimatedPosition();
    }

    protected static void ctre_log_pose(String name, Pose2d pose) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    }
    protected static void ctre_log_pose(String name, Translation2d pose) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY()});
    }

    public Rotation2d get_heading() {
        return get_pose().getRotation();
    }

    public Rotation2d get_raw_pigeon_rotation() {
        return pig.getRotation2d();
    }

    public ChassisSpeeds get_speeds() {
        return kin.to_chassis_speeds(current_states);
    }

    public ChassisSpeeds get_field_relative_speeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(get_speeds(), get_heading());
    }

    public void init_elastic_swerve_vis() {
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty("Front Left Angle", () -> modules[1].get_state().theta_rad, null);
                builder.addDoubleProperty("Front Left Velocity", () -> modules[1].get_state().speed_mps, null);
                builder.addDoubleProperty("Front Right Angle", () -> modules[0].get_state().theta_rad, null);
                builder.addDoubleProperty("Front Right Velocity", () -> modules[0].get_state().speed_mps, null);
                builder.addDoubleProperty("Back Left Angle", () -> modules[3].get_state().theta_rad, null);
                builder.addDoubleProperty("Back Left Velocity", () -> modules[3].get_state().speed_mps, null);
                builder.addDoubleProperty("Back Right Angle", () -> modules[2].get_state().theta_rad, null);
                builder.addDoubleProperty("Back Right Velocity", () -> modules[2].get_state().speed_mps, null);
                builder.addDoubleProperty("Robot Angle", () -> get_heading().getRadians(), null);
            }
        });
    }

    @Override
    public void configure() {
        modules[0].configure();
        modules[1].configure();
        modules[2].configure();
        modules[3].configure();
    }
    
}
