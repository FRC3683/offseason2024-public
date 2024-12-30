package frc.robot.utils.controls;

import static frc.robot.constants.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.config.swerve;
import frc.robot.config.swerve.module_config;
import frc.robot.utils.can_savior;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.talon_temps_safety;
import frc.robot.utils.controls.swerve_kin2.module_state;

public class swerve_module implements configurable {

    enum drive_request {
        PID,
        RAW_VOLTAGE
    }

    public swerve_module(module_config module_config) {
        this.nt_name = module_config.name;
        this.module_config = module_config;
        double hard_coded_offset = Preferences.getBoolean("comp", true) ? module_config.comp_abs_offset : module_config.prac_abs_offset;
        abs_offset = Preferences.getDouble(nt_name+"_abs_rad", hard_coded_offset);

        drive = new TalonFX(module_config.can_drive, config.drive_canbus);
        turn = new TalonFX(module_config.can_turn, config.drive_canbus);
        can_savior.add_talons(drive, turn);
        abs = new DutyCycleEncoder(module_config.dio_abs);
        abs.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);

        drive_pos_signal = drive.getPosition();
        drive_vel_signal = drive.getVelocity();
        turn_pos_signal = turn.getPosition();
        turn_vel_signal = turn.getVelocity();

        turn.setSafetyEnabled(false);
        drive.setSafetyEnabled(false);

        BaseStatusSignal.setUpdateFrequencyForAll(constants.swerve.odom_freq, 
            drive_pos_signal,
            drive_vel_signal,
            turn_pos_signal,
            turn_vel_signal,
            drive.getMotorVoltage()
        );

        BaseStatusSignal.setUpdateFrequencyForAll(4, 
            drive.getClosedLoopReference(),
            turn.getClosedLoopReference()
        );

        drive_temps = new talon_temps_safety(drive, nt_name+"_drive", 45, 70);
        turn_temps = new talon_temps_safety(turn, nt_name+"_turn", 45, 70);

        ParentDevice.optimizeBusUtilizationForAll(drive, turn);

        reset_turning();
    }

    public void init(TimedRobot robot) {
        robot.addPeriodic(this::periodic, RobotBase.isReal() ? 0.25 : 0.02, 0.125); // 4hz, 125ms offset
        robot.addPeriodic(this::one_hz_periodic, 0.73); // 1hz
        if(RobotBase.isSimulation()) {
            turn_sim.init(robot, 200);
            drive_sim.init(robot, 200);
        }
    }

    private void periodic() {
        var base = "swerve/"+nt_name+"_";
        SmartDashboard.putNumber(base + "abs", Math.toDegrees(get_abs()));
        SmartDashboard.putBoolean(base+"abs?", has_abs());
        if(config.dev) {
            SmartDashboard.putNumber(base+"coupling", get_drive_position_rotations() / get_turning_position_rotations());
            SmartDashboard.putNumber(base+"desired_mps", desired_state.speed_mps);
        }
        drive_temps.periodic();
        turn_temps.periodic();
    }

    private void one_hz_periodic() {
        if(!zeroed) {
            reset_turning();
        }
    }

    public boolean toasty() {
        return turn_temps.toasty() || drive_temps.toasty();
    }

    public boolean burning() {
        return turn_temps.cutoff() || drive_temps.cutoff();
    }

    public void save_abs_offset(double theta_raw) {
        abs_offset = theta_raw;
        Preferences.setDouble(nt_name+"_abs_rad", theta_raw);
    }

    public double get_abs_raw() {
        return abs.getAbsolutePosition() * Math.PI * 2;
    }

    public double get_abs() {
        return get_abs_raw() - abs_offset;
    }

    public boolean has_abs() {
        return abs.isConnected();
    }

    public void reset_turning() {
        if(!has_abs()) {
            DriverStation.reportWarning("MISSING ABSOLUTE ENCODER " + nt_name, false);
            zeroed = false;
            return;
        }
        turn.setPosition(get_abs() / (Math.PI * 2.0));
        zeroed = true;
    }

    public double get_turning_position_rotations() {
        if(RobotBase.isSimulation()) {
            return turn_sim.get_position();
        }
        return BaseStatusSignal.getLatencyCompensatedValue(turn_pos_signal, turn_vel_signal);
    }

    public double get_turning_velocity_rotations_ps() {
        if(RobotBase.isSimulation()) {
            return turn_sim.get_velocity();
        }
        return turn_vel_signal.getValue();
    }

    public double get_drive_position_rotations() {
        if(RobotBase.isSimulation()) {
            return drive_sim.get_position();
        }
        return BaseStatusSignal.getLatencyCompensatedValue(drive_pos_signal, drive_vel_signal) + (get_turning_position_rotations() * constants.swerve.module.couple_ratio);
    }

    public double get_drive_position_m() {
        return Units.rotationsToRadians(get_drive_position_rotations()) * constants.swerve.wheel_radius;
    }

    public double get_drive_velocity_rotations_ps() {
        double raw = RobotBase.isSimulation() ? drive_sim.get_velocity() : drive_vel_signal.getValue();
        return raw - (get_turning_velocity_rotations_ps() * constants.swerve.module.couple_ratio);
    }

    public double get_drive_velocity_mps() {
        return Units.rotationsToRadians(get_drive_velocity_rotations_ps()) * constants.swerve.wheel_radius;
    }

    public SwerveModulePosition odom() {
        cached_position.angle = Rotation2d.fromRotations(get_turning_position_rotations());
        cached_position.distanceMeters = get_drive_position_m();
        return cached_position;
    }

    public module_state get_state() {
        return current_state;
    }

    public void update_state() {
        current_state.speed_mps = get_drive_velocity_mps();
        current_state.theta_rad = Units.rotationsToRadians(get_turning_position_rotations());
        current_state.omega_radps = Units.rotationsToRadians(get_turning_velocity_rotations_ps());
    }

    public module_state get_desired_state() {
        return desired_state;
    }

    private void set_speed(double speed_mps, drive_request drive_req) {
        if(last_vel == speed_mps) {
            return;
        }
        last_vel = speed_mps;
        double speed_rotations_ps = Units.radiansToRotations(speed_mps / constants.swerve.wheel_radius);
        drive_control_out.withVelocity(speed_rotations_ps);
        if(RobotBase.isSimulation()) {
            drive_sim.set_control(drive_control_out);
        } else {
            if(drive_req == drive_request.RAW_VOLTAGE) {
                drive.setControl(drive_raw_out.withOutput(speed_mps / constants.swerve.max_module_speed_mps * 12.0));
            } else {
                drive.setControl(drive_control_out);
            }
        }
    }

    private void set_turn(double theta_rad, double omega_radps) {
        if(last_ang == theta_rad) {
            return;
        }
        last_ang = theta_rad;
        turn_control_out.withPosition(theta_rad / (Math.PI * 2.0)).withVelocity(omega_radps / (Math.PI * 2.0));
        if(RobotBase.isSimulation()) {
            turn_sim.set_control(turn_control_out);
        } else {
            turn.setControl(turn_control_out);
        }
    }

    public void stop() {
        if(RobotBase.isSimulation()) {
            drive_sim.stop_motor();
            turn_sim.stop_motor();
        } else {
            drive.stopMotor();
            turn.stopMotor();
        }
    }

    public void set_state(module_state state, boolean optimize, drive_request drive_req) {
        desired_state.speed_mps = state.speed_mps;
        desired_state.theta_rad = state.theta_rad;
        desired_state.omega_radps = state.omega_radps;
        final boolean test_wiggle_reduction = false;
        if(test_wiggle_reduction) {
            desired_state.speed_mps = 0;
            desired_state.theta_rad = current_state.theta_rad + 0.02;
        }

        if(optimize) {
            desired_state.optimize(Rotation2d.fromRotations(get_turning_position_rotations()));
        }

        set_turn(desired_state.theta_rad, desired_state.omega_radps);

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double azimuth_err = desired_state.theta_rad - Units.rotationsToRadians(get_turning_position_rotations());
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosine_scalar = Math.cos(azimuth_err);
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if(cosine_scalar < 0.0) {
            cosine_scalar = 0.0;
        }

        desired_state.speed_mps *= cosine_scalar;

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        double azimuth_turn_rotations_ps = get_turning_velocity_rotations_ps();
        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        double drive_rate_back_out = Units.rotationsToRadians(azimuth_turn_rotations_ps * constants.swerve.module.couple_ratio) * constants.swerve.wheel_radius;

        set_speed(desired_state.speed_mps - drive_rate_back_out, drive_req);

        state.speed_mps = desired_state.speed_mps;
        state.theta_rad = desired_state.theta_rad;
        state.omega_radps = desired_state.omega_radps;
    }

    public void sysid(double voltage) {
        set_turn(0, 0);
        drive.setControl(voltage_out.withOutput(voltage));
    }

    public void coast() {
        if(RobotBase.isSimulation()) {
            drive_sim.stop_motor();
            turn_sim.stop_motor();
        } else {
            drive.setControl(new CoastOut());
            turn.setControl(new CoastOut());
        }
    }

    public final String nt_name;
    public final talon_temps_safety drive_temps, turn_temps;
    private final TalonFX drive, turn;
    private final DutyCycleEncoder abs;
    public final StatusSignal<Double> drive_pos_signal, turn_pos_signal;
    public final StatusSignal<Double> drive_vel_signal, turn_vel_signal;
    private final PositionVoltage turn_control_out = new PositionVoltage(0).withEnableFOC(config.swerve.pro).withSlot(0);
    private final VelocityVoltage drive_control_out = new VelocityVoltage(0).withEnableFOC(config.swerve.pro).withSlot(0);
    private final VoltageOut drive_raw_out = new VoltageOut(0);
    private final VoltageOut voltage_out = new VoltageOut(0);
    private final module_config module_config;
    private final sim_talon turn_sim = new sim_talon(DCMotor.getKrakenX60Foc(1), module.steer_ratio, 0.002)
        .with_slot(0, swerve.sim_turn_configs).with_continuous_wrap();
    private final sim_talon drive_sim = new sim_talon(DCMotor.getKrakenX60Foc(1), module.drive_ratio, 0.1)
        .with_slot(0, swerve.sim_drive_configs);

    private boolean zeroed = false;
    private double abs_offset;
    private double last_vel, last_ang;
    private module_state desired_state = new module_state(0, 0, 0);
    private module_state current_state = new module_state(0, 0, 0); // only updated on get_state
    private SwerveModulePosition cached_position = new SwerveModulePosition();

    @Override
    public void configure() {
        drive.clearStickyFaults();
        lazy_ctre.lazy_config(drive, config.swerve.drive_configs(module_config.drive_inverted));
        turn.clearStickyFaults();
        lazy_ctre.lazy_config(turn, config.swerve.turn_configs(module_config.turn_inverted));
    }
}
