package frc.robot.utils.controls;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class sim_talon {
    private final DCMotorSim sim;
    private final PIDController[] pid = {
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0),
    };
    private final SimpleMotorFeedforward[] ff = {
        new SimpleMotorFeedforward(0, 0),
        new SimpleMotorFeedforward(0, 0),
        new SimpleMotorFeedforward(0, 0),
        new SimpleMotorFeedforward(0, 0),
    };
    private double control_dts;

    private CoastOut coast = new CoastOut();
    private ControlRequest req = coast;

    public sim_talon(DCMotor motor, double gear_ratio, double moment_of_inertia) {
        sim = new DCMotorSim(motor, gear_ratio, moment_of_inertia);
    }

    public void init(TimedRobot robot, int control_hz) {
        control_dts = 1.0 / (double)control_hz;
        robot.addPeriodic(this::periodic, control_dts);
    }

    private void periodic() {
        if(req.getClass().equals(CoastOut.class)) {
            SmartDashboard.putString("test", "coast");
            sim.setInputVoltage(0);
        } else if(req.getClass().equals(PositionVoltage.class)) {
            var control = (PositionVoltage)req;
            double feedforward = ff[control.Slot].calculate(control.Velocity) + control.FeedForward;
            double feedback = pid[control.Slot].calculate(sim.getAngularPositionRotations(), control.Position);
            sim.setInputVoltage(feedforward + feedback);
            req = control;
        } else if(req.getClass().equals(VelocityVoltage.class)) {
            var control = (VelocityVoltage)req;
            double feedforward = ff[control.Slot].calculate(control.Velocity, control.Acceleration) + control.FeedForward;
            double feedback = pid[control.Slot].calculate(Units.radiansToRotations(sim.getAngularVelocityRadPerSec()), control.Velocity);
            double output = feedforward + feedback;
            if(Math.abs(output) < 0.1) {
                output = 0;
            }
            sim.setInputVoltage(output);
            req = control;
        } else {
            req = new CoastOut();
        }
        if(Math.abs(sim.getAngularVelocityRPM()) < 10) { // wierd things happen
            sim.setState(sim.getAngularPositionRad(), 0);
        }
        sim.update(control_dts);
    }

    public sim_talon with_slot(int slot, Slot0Configs configs) {
        pid[slot].setPID(configs.kP, configs.kI, configs.kD);
        ff[slot] = new SimpleMotorFeedforward(configs.kS, configs.kV, configs.kA);
        return this;
    }

    public sim_talon with_continuous_wrap() {
        pid[0].enableContinuousInput(-0.5, 0.5);
        pid[1].enableContinuousInput(-0.5, 0.5);
        pid[2].enableContinuousInput(-0.5, 0.5);
        pid[3].enableContinuousInput(-0.5, 0.5);
        return this;
    }

    public void set_position(double rotations) {
        sim.setState(Units.rotationsToRadians(rotations), 0);
    }

    public double get_position() {
        return sim.getAngularPositionRotations();
    }

    public double get_velocity() {
        return Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    }

    public void set_control(ControlRequest req) {
        this.req = req;
    }

    public void stop_motor() {
        req = coast;
    }
}
