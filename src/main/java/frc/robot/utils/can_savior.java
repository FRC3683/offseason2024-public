package frc.robot.utils;

import java.util.SortedSet;
import java.util.TreeSet;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;
import frc.robot.robot;

public class can_savior {

    private static abstract class can_device {
        int order;

        void set_order() {
            for(int i=0; i < loop.length; ++i) {
                if(id() == loop[i]) {
                    order = i;
                    break;
                }
            }
        }

        abstract boolean is_connected();
        abstract int id();
    }

    private static class talon extends can_device {
        private CoreTalonFX device;

        private talon(CoreTalonFX device) {
            this.device = device;
            set_order();
        }

        @Override
        boolean is_connected() {
            return device.getVersion().getStatus().isOK(); // MAYBE??!?!?!?!?
        }

        @Override
        int id() {
            return device.getDeviceID();
        }
    }

    private static class cancoder extends can_device {
        private CoreCANcoder device;

        private cancoder(CoreCANcoder device) {
            this.device = device;
            set_order();
        }

        @Override
        boolean is_connected() {
            return device.getVersion().getStatus().isOK(); // MAYBE??!?!?!?!?
        }

        @Override
        int id() {
            return device.getDeviceID();
        }
    }

    static int[] loop = {};
    static SortedSet<can_device> devices = new TreeSet<>((can_device d1, can_device d2) -> Integer.compare(d1.order, d2.order));

    public static void init(int[] loop) {
        can_savior.loop = loop;
    }
    
    public static void begin(robot robot) {
        robot.addPeriodic(can_savior::periodic, constants.control_dts, 0.05);
    }

    public static void add_talons(CoreTalonFX... add) {
        for(var device : add) {
            devices.add(new talon(device));
        }
    }

    public static void add_cancoder(CoreCANcoder... add) {
        for(var device : add) {
            devices.add(new cancoder(device));
        }
    }

    private static void periodic() {
        int can_break = -1;
        for(var device : devices) {
            if(!device.is_connected()) {
                can_break = device.id();
                break;
            }
        }
        SmartDashboard.putBoolean("can_gucci", can_break == -1);
        SmartDashboard.putNumber("can_break", can_break);
        SignalLogger.writeInteger("can_break", can_break);
    }
}
