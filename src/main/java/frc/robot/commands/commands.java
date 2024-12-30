package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.config.swerve;
import frc.robot.utils.dave_led;
import frc.robot.utils.math_utils;

public class commands {

    public static Command get_note(robot robot) {
        var swerve = robot.swerve;
        return Commands.parallel(
            swerve.turn(() -> 0.0),
            swerve.strafe_field_relative(() -> {
                if(!robot.swerve.sees_note()){// || robot.intake.sees_note()) {
                    return new ChassisSpeeds(0.7, 0, 0);
                }
                var assist = robot.swerve.calc_intake_assist(new ChassisSpeeds(0.1, 0, 0));
                var note_dist = swerve.note_dist();
                final double kP = 0.8;
                assist.vxMetersPerSecond = math_utils.clamp(0, 1.1, note_dist * kP) + 0.7;
                return assist;
            }),
            Commands.repeatingSequence(
                robot.leds.flag_auto_intake_sees.run()
                    .until(() -> !robot.swerve.sees_note()),
                robot.leds.flag_auto_intake_blind.run()
                    .until(() -> robot.swerve.sees_note())
            )
        );

    }

    public static Command climb_snap(robot robot) {
        return robot.swerve.snap(() -> {
            var pose = robot.swerve.get_pose();
            return constants.field.stage_normal(pose.getTranslation()).getDegrees();
        });
    }

    public static Command default_strafe(robot robot, Supplier<Translation2d> ctrl_strafe, Supplier<Boolean> shot_throttle, 
            Supplier<Boolean> amp_throttle, Supplier<Boolean> intaking, Supplier<Boolean> climbing)
    {
        final String pref_key = "drive_speed", pref_key_intake_assist = "intake_assist";

        Preferences.initBoolean(pref_key_intake_assist, false);
        Preferences.initDouble(pref_key, 10);

        return robot.swerve.strafe_robot_relative(() -> {
            double throttle = 1;
            // if(climb_prep.isScheduled()) {
            //     throttle = constants.swerve.climb_throttle;
            // } else 
            if(shot_throttle.get()) {
                throttle *= constants.swerve.shot_throttle;
            } else if(amp_throttle.get()) {
                throttle *= constants.swerve.amp_throttle;
            } else if(climbing.get()) {
                throttle *= constants.swerve.climb_throttle;
            }

            var chassis_speeds = robot.swerve.get_speeds();
            ChassisSpeeds assist = new ChassisSpeeds();
            boolean should_assist = Preferences.getBoolean(pref_key_intake_assist, true) 
                && intaking.get();// && !robot.shooter.has_note();
            if(should_assist) {
                assist = robot.swerve.calc_intake_assist(chassis_speeds);
            }

            double pref = Preferences.getDouble(pref_key, 10);
            double scale = math_utils.remap(1, 10, pref, constants.swerve.max_speed_mps*0.1, constants.swerve.max_speed_mps);
            var speeds = ctrl_strafe.get().times(scale * throttle);

            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                    speeds.getX(),
                    speeds.getY(),
                    0),
                robot.swerve.get_heading().plus(forwards())
            ).plus(assist);
        });
    }

    public static Command default_omega(robot robot, Supplier<Double> ctrl_turn) {
        final String pref_key = "turn_speed";
        Preferences.initDouble(pref_key, 7);
        double max_omega = robot.swerve.get_max_chassis_radps() * 0.95;

        return robot.swerve.turn(() -> {
            double theta_in = ctrl_turn.get();
            double pref = Preferences.getDouble(pref_key, 7);
            double scale = math_utils.remap(1, 10, pref, max_omega * 0.3, max_omega);
            return theta_in * scale;
        });
    }

    public static Rotation2d forwards() {
        return Rotation2d.fromDegrees(robot.is_red() ? 180 : 0);
    }

}
