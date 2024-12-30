package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.swerve;
import frc.robot.utils.LL_connected_check;
import frc.robot.utils.dave_led;
import frc.robot.utils.voltage_warning;

public class leds extends dave_led {

    private final LL_connected_check intake = new LL_connected_check(config.LL_intake);
    private final LL_connected_check pose = new LL_connected_check(config.LL_shooter);
    private final swerve swerve;

    public final flag flag_configuring = new flag();
    public final flag flag_operator_flash = new flag();
    public final flag flag_auto_intake_sees = new flag();
    public final flag flag_auto_intake_blind = new flag();

    public leds(TimedRobot robot, swerve swerve) {
        super(config.pwm_leds, config.leds.length, robot);
        this.swerve = swerve;

        robot.addPeriodic(this::periodic, constants.control_dts);
    }

    private void handle_flags() {
        if(flag_configuring.get()) {
            set_color(0, length/2, clr_blue);
            set_color(length/2, length, clr_purple);
            return;
        }
        if(flag_operator_flash.get()) {
            set_fast_flicker(clr_yellow);
            return;
        }

        if(DriverStation.isDisabled()) {
            return;
        }

        if(flag_auto_intake_blind.get()) {
            set_fast_flicker(clr_red);
            return;
        }
        if(flag_auto_intake_sees.get()) {
            set_color(clr_purple);
            return;
        }

        set_color(clr_green);
    }

    @Override
    protected void periodic() {
        if(DriverStation.isDisabled()) {
            set_color(0, length, clr_off);

            Color8Bit disabled_color = clr_purple;
            if(DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
                switch(DriverStation.getAlliance().get()) {
                    case Blue: disabled_color = clr_blue; break;
                    case Red:  disabled_color = clr_red; break;
                }
            }

            if(DriverStation.isTest()) {
                rainbow(5);
            } else if(DriverStation.isAutonomous()) {
                set_single(0, disabled_color);
            }

            set_trail(disabled_color);

        }

        handle_flags();


        // most important, override other led codes
        if(RobotBase.isReal()) {
            if(!voltage_warning.check()) {
                set_checker(0, length, clr_red, clr_blue);
            }
            if(DriverStation.isEStopped()) {
                set_checker(0, length, clr_purple, clr_red);
            }
            if(!swerve.has_abs()) {
                set_checker(0, 4, clr_cyan, clr_oramge);
            }
            if(!DriverStation.isJoystickConnected(0)
                    || (DriverStation.isDisabled() && !DriverStation.isJoystickConnected(1))) {
                set_checker(4, 8, clr_white, clr_red);
            }
            if(!intake.connected() || !pose.connected()) {
                set_checker(12, 16, clr_green, clr_red);
            }
            if(swerve.toasty_motors()) {
                set_checker(8, 12, clr_red, clr_yellow);
            }
        }
        set_single(length-1, LimelightHelpers.getTV(config.LL_intake) ? clr_green : clr_red);
        set_single(length-2, LimelightHelpers.getTV(config.LL_shooter) ? clr_green : clr_red);
    }

}
