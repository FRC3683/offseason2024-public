package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class talon_temps_safety {

    public static enum state {
        CHILLIN,
        TOASTY,
        CUTOFF
    }

    public talon_temps_safety(TalonFX device, String dashboard_name, double warning, double error) {
        this.dashboard_name = "swerve/"+dashboard_name+"_";
        device_temp_signal = device.getDeviceTemp();
        ancillary_temp_signal = device.getAncillaryDeviceTemp();
        processor_temp_signal = device.getProcessorTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(4, device_temp_signal, ancillary_temp_signal, processor_temp_signal);

        this.warning_temp = warning;
        this.error_temp = error;
        temp = state.CHILLIN;
    }

    public void periodic() {
        if(RobotBase.isSimulation()) {
            return;
        }

        double device_temp = device_temp_signal.refresh().getValue();
        double proc_temp = processor_temp_signal.refresh().getValue();

        boolean chillin = device_temp <= warning_temp 
                        && proc_temp <= warning_temp;

        boolean error = device_temp >= error_temp || proc_temp >= error_temp;

        temp = error ? state.CUTOFF : (chillin ? state.CHILLIN : state.TOASTY);

        SmartDashboard.putNumber(dashboard_name + "device temp", device_temp);
        SmartDashboard.putNumber(dashboard_name + "processor temp", proc_temp);
    }

    public state get() {
        return temp;
    }

    public boolean cutoff() {
        return temp == state.CUTOFF;
    }

    public boolean toasty() {
        return temp == state.TOASTY || temp == state.CUTOFF;
    }

    private final StatusSignal<Double> device_temp_signal, ancillary_temp_signal, processor_temp_signal;
    private final double warning_temp, error_temp;
    private final String dashboard_name;
    private state temp;
}
