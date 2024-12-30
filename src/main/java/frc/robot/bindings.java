package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.commands;
import frc.robot.commands.drive_goofy;
import frc.robot.subsystems.swerve;
import frc.robot.utils.field_util;
import frc.robot.utils.oi;

public final class bindings {

    public static void configure_bindings(robot robot) {
        configure_auto(robot);

        final var strafe_input_shaping = new oi.shaping_chooser("strafe_input_shaping");
        final var turn_input_shaping = new oi.shaping_chooser("turn_input_shaping");

        // CONTROLS
        Supplier<Translation2d> ctrl_strafe = () -> oi.vec_deadband(oi.get_left_stick(oi.driver), strafe_input_shaping::shape);
        Supplier<Double> ctrl_turn = () -> oi.deadband_precise(-oi.driver.getRightX(), turn_input_shaping::shape);
        var ctrl_reset_heading = oi.cmd_driver.povDown();
        var ctrl_lineup_amp = oi.cmd_driver.x(); // Line up To AMP
        Supplier<Boolean> ctrl_score_amp = () -> (oi.driver.getLeftTriggerAxis() > 0.5); // Score Amp
        var ctrl_intake_down = oi.cmd_driver.rightTrigger();
        var ctrl_intake_stowed = oi.cmd_driver.start().or(ctrl_intake_down); // right middle // or right trigger
        var ctrl_shoot = oi.cmd_driver.rightBumper();
        var ctrl_backfeed = oi.cmd_driver.leftBumper();
        var ctrl_spit = oi.cmd_driver.back(); // left middle
        var ctrl_auto_pickup = oi.cmd_driver.b();
        var ctrl_climb_prep = oi.cmd_driver.y();
        var ctrl_uppies = oi.cmd_driver.povLeft();
        var ctrl_hp_intake = oi.cmd_driver.a();

        // XKEYS
        var ctrl_zero_arm = oi.cmd_xkeys.button(10);
        var ctrl_manual_lob_center = oi.cmd_xkeys.button(13);
        var ctrl_manual_lob_edge = oi.cmd_xkeys.button(14);
        var ctrl_arm_manual_up = oi.cmd_xkeys.button(15);
        var ctrl_arm_manual_down = oi.cmd_xkeys.button(16);
        var ctrl_tap = oi.cmd_xkeys.button(19);
        var ctrl_tapnt = oi.cmd_xkeys.button(20);
        var ctrl_zero_intake = oi.cmd_xkeys.button(21);
        var ctrl_flash_leds = oi.cmd_xkeys.button(8);
        var ctrl_swerve_abs = oi.cmd_xkeys.button(9);

        swerve swerve = robot.swerve;

        
        var get_note = commands.get_note(robot);

        final boolean goofy = false;
        if(goofy) {
            var cmd = drive_goofy.drive_rc_car(
                swerve, () -> -oi.driver.getLeftY(), () -> -oi.driver.getRightX(), 3, Units.degreesToRadians(80));
            swerve.omega_subsystem.setDefaultCommand(cmd);
            swerve.strafe_subsystem.setDefaultCommand(cmd);
        } else {
            robot.swerve.strafe_subsystem.setDefaultCommand(
                commands.default_strafe(robot, ctrl_strafe, () -> false, () -> false, 
                    () -> ctrl_intake_down.getAsBoolean() || ctrl_intake_stowed.getAsBoolean(), () -> false));

            robot.swerve.omega_subsystem.setDefaultCommand(
                Commands.repeatingSequence(
                    robot.swerve.maintain_heading().until(() -> Math.abs(ctrl_turn.get()) > 0.005),
                    commands.default_omega(robot, ctrl_turn).until(new Trigger(() -> Math.abs(ctrl_turn.get()) < 0.005).debounce(0.5))
            ));
        }

        ctrl_swerve_abs.onTrue(Commands.runOnce(() -> {
            swerve.reset_encoders();
        }).ignoringDisable(true));

        ctrl_auto_pickup.whileTrue(get_note);

        ctrl_reset_heading.onTrue(Commands.runOnce(() -> {
            swerve.zero_heading(commands.forwards());
        }).ignoringDisable(true));

        final var spot = field_util.field_size_m.div(2);
        ctrl_shoot.whileTrue(swerve.snap_trans(() -> spot, 0));


        ctrl_flash_leds.whileTrue(robot.leds.flag_operator_flash.run());
    }

    private static void configure_auto(robot robot) {

    }
}
