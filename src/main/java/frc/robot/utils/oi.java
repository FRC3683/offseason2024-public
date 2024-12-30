package frc.robot.utils;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class oi {
    public static final double default_deadzone = 0.08;
    public static final double default_rumble_power = 0.25;

    public static final CommandXboxController cmd_driver = new CommandXboxController(0);
    public static final XboxController driver = cmd_driver.getHID();
    public static final CommandGenericHID cmd_xkeys = new CommandGenericHID(1);
    public static final GenericHID xkeys = cmd_xkeys.getHID();

    public static void rumble_for(double seconds) { rumble_for(seconds, default_rumble_power).schedule(); }

    public static Command rumble_for(double seconds, double power) {
        return Commands.startEnd(() -> {
            driver.setRumble(RumbleType.kBothRumble, power);
        }, () -> {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }).withTimeout(seconds);
    }

    // up-left positive
    public static Translation2d get_left_stick(final XboxController controller) {
        return new Translation2d( -controller.getLeftY(), -controller.getLeftX() );
    }

    // up-left positive
    public static Translation2d get_right_stick(final XboxController controller) {
        return new Translation2d( -controller.getRightY(), -controller.getRightX() );
    }
    
    public static Translation2d vec_deadband(final Translation2d vec_in, final double deadzone, final double max_len, final DoubleUnaryOperator shaping_function) {
        double len = vec_in.getNorm();
        if(len <= deadzone) {
            return new Translation2d(0, 0);
        }
        var theta = vec_in.getAngle();
        double new_len = shaping_function.applyAsDouble((len - deadzone) / (max_len - deadzone));
        return new Translation2d(Math.min(new_len, max_len), theta);
    }

    public static Translation2d vec_deadband(final Translation2d vec_in, final DoubleUnaryOperator shaping_function) {
        return vec_deadband(vec_in, default_deadzone, 1, shaping_function);
    }
    
    public static double deadband_precise(final double d_in, final double deadzone, final double max_len, final DoubleUnaryOperator shaping_function) {
        final double sign = d_in < 0 ? -1 : (d_in > 0 ? 1 : 0);
        final double d = Math.abs(d_in);
        if(d < deadzone) {
            return 0;
        }
        double deadzoned = (d - deadzone) / (max_len - deadzone);
        return shaping_function.applyAsDouble(Math.abs(deadzoned)) * sign;
    }

    public static double deadband_precise(final double d_in, final DoubleUnaryOperator shaping_function) {
        return deadband_precise(d_in, default_deadzone, 1, shaping_function);
    }

    public static class shaping_chooser {
        private final SendableChooser<String> chooser;
        private final String linear = "linear", squared = "squared", cubic = "cubic", custom = "custom";
        private final String[] options = { linear, squared, cubic, custom };

        public shaping_chooser(String pref_key) {
            chooser = new SendableChooser<String>();
            for(var option : options) {
                chooser.addOption(option, option);
            }
            var pref = Preferences.getString(pref_key, squared);
            chooser.setDefaultOption(pref, pref);
            chooser.onChange((String option) -> {
                Preferences.setString(pref_key, option);
            });
            SmartDashboard.putData(chooser);
        }

        public static double custom(double x) {
            if(x <= 0.5) {
                return 0.25 * x;
            }
            return 0.904 * math_utils.quint(x) + 0.09675;
        }

        public double shape(double x) {
            switch (chooser.getSelected()) {
                case squared: return math_utils.sq(x);
                case cubic: return math_utils.cube(x);
                case custom:
                    return custom(x);
                case linear:
                    return x;
                default:
                    return x;
            }
        }
    }
}
