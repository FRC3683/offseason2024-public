package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class voltage_warning {

    private static double threshold_comp = 12.6;
    private static double threshold_prac = 12.1;

    public static void set_thresholds(double voltage_warning_threshold_comp, double voltage_warning_threshold_prac) {
        threshold_comp = voltage_warning_threshold_comp;
        threshold_prac = voltage_warning_threshold_prac;
    }

    public static void add_nt_listener(TimedRobot robot, int hz) {
        robot.addPeriodic(() -> {
            SmartDashboard.putBoolean("voltage_status", check());
        }, 1.0 / hz);
    }

    public static boolean check() {
        if(DriverStation.isEnabled()) {
            return true; // dont give errors when enabled, voltage drops frequently while robot is running, totally ok
        }
        double voltage_warning_threshold = DriverStation.isFMSAttached() ? threshold_comp : threshold_prac;
        return RobotController.getBatteryVoltage() >= voltage_warning_threshold;
    }
}
