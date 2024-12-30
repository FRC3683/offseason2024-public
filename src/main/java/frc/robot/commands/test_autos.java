package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.utils.controls.swerve_kin2.module_state;

import static frc.robot.utils.auto_utils.*;

import java.util.Arrays;

public class test_autos {


    /*
     * test decel distance
     */
    public static Command straight(robot robot, double dist, double vel) {
        return no_vision(
        Commands.sequence(
            Commands.runOnce(() -> {
                robot.swerve.reset_pose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            }),
            Commands.deadline(
                robot.swerve.strafe_line(new Translation2d(dist, 0), Rotation2d.fromDegrees(0), 0.05, 0),
                robot.swerve.snap(0),
                new Command() {
                    double prev_speed = 0;
                    @Override
                    public boolean isFinished() {
                        double x_speed = robot.swerve.get_speeds().vxMetersPerSecond;
                        if(prev_speed > constants.swerve.max_speed_mps - 0.02 && x_speed < prev_speed) {
                            SmartDashboard.putNumber("s"+dist+" test decel point", robot.swerve.get_pose().getX());
                            return true;
                        }
                        prev_speed = x_speed;
                        return false;
                    }
                }
            ),
            Commands.runOnce(() -> {
                SmartDashboard.putNumber("s"+dist+" test", robot.swerve.get_pose().getX());
            })
        ), robot.swerve);
    }

    /*
     * TO CALIBRATE:
     * set constants.swerve.k = 1
     * push code
     * run command
     * set constants.swerve.k = Preferences "wheel_calibration"
     * push code
     * run command, verify Preferences "wheel_calibration" is very close to 1
     */
    public static Command clibrate_wheel_radius(robot robot) {
        final double buffer_seconds = 0.5;
        final double calibration_seconds = 6;

        final ChassisSpeeds stopped = new ChassisSpeeds(0, 0, 0);
        final String pref_key = "wheel_calibration";
        Preferences.initDouble(pref_key, 1);
        final var wrapper = new Object() {
            Rotation2d initial_heading;
            SwerveModulePosition[] initial_positions;
        };

        return Commands.sequence(
            robot.swerve.turn(() -> Math.PI).withTimeout(0.1),
            robot.swerve.strafe_omega(() -> stopped).withTimeout(buffer_seconds),
            Commands.runOnce(() -> {
                wrapper.initial_heading = robot.swerve.get_raw_pigeon_rotation();
                wrapper.initial_positions = robot.swerve.copy_module_positions();
            }),
            robot.swerve.turn(() -> Math.PI).withTimeout(calibration_seconds * 0.666),
            robot.swerve.turn(() -> Math.PI * 2).withTimeout(calibration_seconds * 0.333),
            robot.swerve.strafe_omega(() -> stopped).withTimeout(buffer_seconds),
            Commands.runOnce(() -> {
                var heading = robot.swerve.get_raw_pigeon_rotation();
                var positions = robot.swerve.copy_module_positions();
                var encoder_delta_radii = new double[positions.length];
                for(int i = 0; i < positions.length; ++i) {
                    encoder_delta_radii[i] = Math.abs(positions[i].distanceMeters - wrapper.initial_positions[i].distanceMeters) / robot.swerve.chassis_radius(i);
                }
                double encoder_delta_rad = Arrays.stream(encoder_delta_radii).sum() / encoder_delta_radii.length;
                double pig_delta_rad = heading.getRadians() - wrapper.initial_heading.getRadians();
                double k = pig_delta_rad / encoder_delta_rad;
                Preferences.setDouble(pref_key, k);
                Preferences.setDouble("foo", encoder_delta_rad);
                Preferences.setDouble("bar", pig_delta_rad);
            })
        );
    }

    /*
     * check for skew
     */
    public static Command skew_test(robot robot, double speed, double theta, double seconds) {
        return Commands.parallel(
            robot.swerve.strafe_field_relative(() -> new ChassisSpeeds(speed, 0, 0)),
            robot.swerve.turn(() -> theta)
        ).withTimeout(seconds);
    }

    /*
     * test pid line
     */
    public static Command pid_line_test(robot robot) {
        return Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(2, 2, Rotation2d.fromDegrees(0))),
            robot.swerve.strafe_line(new Translation2d(6, 4), Rotation2d.fromDegrees(0), 4, 0.05, 0),
            robot.swerve.strafe_line(new Translation2d(2, 4), Rotation2d.fromDegrees(150), 4, 0.05, 0)
        );
    }

    /*
     * test intermediate velocity
     */
    public static Command through_test(robot robot, boolean through) {
        return Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(2, 2, Rotation2d.fromDegrees(0))),
            robot.swerve.strafe_to_point(new Translation2d(4, 2), 4.3, 0.1, through),
            robot.swerve.strafe_to_point(new Translation2d(6, 4), 4.3, 0.05, false)
        );
    }
}
