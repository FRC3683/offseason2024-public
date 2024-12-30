package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.sim.crt_sim;
import frc.robot.utils.math_utils;

import static frc.robot.constants.turret.*;

public class turret extends SubsystemBase {
    private final DutyCycleEncoder mini, minier;
    private final TalonFX motor;
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.2, 0.02);
    private final PIDController pid = new PIDController(0, 0, 0);
    private double target_deg=180, target_degps=0;
    private crt_sim sim;

    public turret() {
        mini = new DutyCycleEncoder(2);
        minier = new DutyCycleEncoder(3);
        mini.setDistancePerRotation(-1);
        mini.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);
        minier.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);
        motor = new TalonFX(14, config.drive_canbus);
        sim = new crt_sim("turret_sim", main_gear, mini_gear, minier_gear, 0.5);
    }

    private double abs_mini() {
        return RobotBase.isSimulation() ? sim.get_enc1() : MathUtil.inputModulus(mini.getAbsolutePosition() - constants.turret.offset_mini, 0, 1);
    }

    private double abs_minier() {
        return RobotBase.isSimulation() ? sim.get_enc2() : MathUtil.inputModulus(minier.getAbsolutePosition() - constants.turret.offset_minier, 0, 1);
    }

    public void periodic() {
        motor.setVoltage(ff.calculate(target_degps) + pid.calculate(get_position_degrees(), target_deg));
        if(RobotBase.isSimulation()) {
            double degps = target_degps + (target_deg - get_position_degrees()) * 4;
            sim.add(degps * constants.control_dts);
        }
        SmartDashboard.putNumber("abs_mini", abs_mini());
        SmartDashboard.putNumber("abs_minier", abs_minier());
        SmartDashboard.putNumber("turret_deg", get_position_degrees());
    }

    public double get_position_degrees() {
        double angle_mini = abs_mini();
        double modulo = (double)constants.turret.mini_gear / (double)constants.turret.main_gear;

        double angle_minier = abs_minier();

        double ratio_mini = (double)constants.turret.mini_gear / (double)constants.turret.main_gear;
        double ratio_minier = (double)constants.turret.minier_gear / (double)constants.turret.main_gear;
        double base = angle_mini * ratio_mini;
        double closest = 0;
        double smallest_diff = 0;
        for(int i = 0; i < 4; ++i) {
            double turret_maybe = base + i * modulo;
            double minier_maybe = MathUtil.inputModulus(turret_maybe / ratio_minier, 0, 1);
            double diff = Math.abs(angle_minier - minier_maybe);
            if(i == 0 || diff < smallest_diff) {
                smallest_diff = diff;
                closest = turret_maybe;
            }
        }
        return Units.rotationsToDegrees(closest);
    }

    public void set_target(double degrees, double degps) {
        target_deg = degrees;
        target_degps = degps;
    }

    public double get_target() {
        return target_deg;
    }

    public void stop() {
        motor.stopMotor();
    }
}
