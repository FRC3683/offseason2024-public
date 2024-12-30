// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.autos;
import frc.robot.commands.test_autos;
import frc.robot.utils.auto_selector;
import frc.robot.utils.can_savior;
import frc.robot.utils.configurable;
import frc.robot.utils.dave_auto;
import frc.robot.utils.oi;
import frc.robot.utils.uptime;
import frc.robot.utils.voltage_warning;
import frc.robot.utils.auto_selector.auto;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.turret;

public final class robot extends TimedRobot {

    public final swerve swerve;
    public final turret turret;
    public final leds leds;
    private final auto_selector selector;
    private Command auto_command;

    private final configurable[] configurables;

    public static boolean is_red() {
        var a = DriverStation.getAlliance();
        return a.isPresent() && a.get().equals(Alliance.Red);
    }

    public robot() {
        super(constants.control_dts);
        can_savior.init(config.can_loop);
        swerve = new swerve(this);
        turret = new turret();
        configurables = new configurable[]{ swerve };
        dave_auto.config(swerve);
        leds = new leds(this, swerve);
        selector = new auto_selector(
            new auto[]{
                auto.from("decel_test3",     () -> autos.decel_test3(this)),
                auto.from("dec20", () -> dave_auto.from(Path.of("dec20.json")).rest()),
                auto.from("decel_test2",     () -> autos.decel_test2(this)),
                auto.from("decel_test",      () -> autos.decel_test(this)),
                new auto("dave_paths test",  () -> autos.app_test()),
                auto.from("swerve test",     () -> autos.test(this)),
            }, new auto[]{
                auto.from("calibrate wheel radius",  () -> test_autos.clibrate_wheel_radius(this)),
                auto.from("swerve PID tune",         () -> test_autos.straight(this, 4, constants.swerve.max_speed_mps)),
                auto.from("calibrate 15m",           () -> test_autos.straight(this, 15, 3)),
                auto.from("ballet",                  () -> test_autos.skew_test(this, 1.5, 2, 4)),
                auto.from("test",                    () -> test_autos.pid_line_test(this)),
            }, swerve::reset_pose
        );
    }

    @Override
    public void robotInit() {
        swerve.reset_encoders();
        bindings.configure_bindings(this);
        debug.add_dashboard_commands(this);
        selector.init();
        voltage_warning.set_thresholds(constants.voltage_warning_threshold_comp, constants.voltage_warning_threshold_prac);
        voltage_warning.add_nt_listener(this, 2);
        Commands.waitSeconds(3).andThen(() -> {
            LimelightHelpers.SetFiducialIDFiltersOverride(config.LL_shooter, config.valid_ids);
        }).schedule();
        // can_savior.begin(this);
        uptime.init(this);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    boolean configured = false;

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        if(!configured) {
            configurable.configure_all(leds.flag_configuring.run(), configurables);
            configured = true;
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        auto_command = selector.get_auto_command();
        if(auto_command != null) {
            auto_command.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        // cancel all ensures any commands scheduled with async() are 
        // stopped before teleop, so default commands can run
        // without this, snap() at the end of an auto will prevent turning during teleop
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        swerve.stop_motors().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
    }

    @Override
    public void testPeriodic() {
        if(oi.driver.getLeftBumperPressed()) {
            turret.set_target(turret.get_target() + 30, 0);
        } else if(oi.driver.getRightBumperPressed()) {
            turret.set_target(turret.get_target() - 30, 0);
        }
    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
