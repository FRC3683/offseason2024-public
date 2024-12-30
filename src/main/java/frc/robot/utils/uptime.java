package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;

public class uptime {
    enum state {
        UPTIME_AUTO_PRACTICE,
        UPTIME_TELE_PRACTICE,
        UPTIME_COMP_FMS
    }

    private static double dts = 0.1;

    public static void init(TimedRobot robot) {
        for(var s : state.values()) {
            Preferences.initDouble(s.name(), 0.5);
        }
        robot.addPeriodic(uptime::periodic, dts, dts/2.0);
    }

    private static void periodic() {
        if(!DriverStation.isEnabled()) {
            return;
        }
        var curr = DriverStation.isFMSAttached() ? state.UPTIME_COMP_FMS
            : (DriverStation.isAutonomous() ? state.UPTIME_AUTO_PRACTICE : state.UPTIME_TELE_PRACTICE);

        double total = Preferences.getDouble(curr.name(), 0);
        Preferences.setDouble(curr.name(), total + dts);
    }
}
