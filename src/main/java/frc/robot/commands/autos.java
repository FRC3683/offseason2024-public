package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.subsystems.swerve;
import frc.robot.utils.dave_auto;

import static frc.robot.utils.auto_utils.*;

import java.nio.file.Path;

public class autos {

    public static Command test(robot robot) {
        return no_vision(
        Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
            robot.swerve.strafe_to_point(new Translation2d(2, 0), 3, 0.15, true),
            robot.swerve.strafe_to_point(new Translation2d(3, 1), 3, 0.05, false)
        ), robot.swerve);
    }

    public static Command decel_test(robot robot) {
        return no_vision(
        Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
            async(robot.swerve.snap(0)),
            robot.swerve.strafe_line(new Translation2d(2, 0), Rotation2d.fromDegrees(0), 0.1, 2),
            robot.swerve.strafe_line(new Translation2d(3, 0), Rotation2d.fromDegrees(0), 2, 0.0, 0)
        ), robot.swerve);
    }

    public static Command decel_test2(robot robot) {
        return no_vision(
        Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
            async(robot.swerve.snap(0)),
            robot.swerve.strafe_line(new Translation2d(3, 0), Rotation2d.fromDegrees(-45), 0.3, 2),
            robot.swerve.strafe_line(new Translation2d(6, 0), Rotation2d.fromDegrees(0), 3, 0.0, 0)
        ), robot.swerve);
    }

    public static Command decel_test3(robot robot) {
        return no_vision(
        Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
            async(robot.swerve.snap(0)),
            robot.swerve.strafe_line(new Translation2d(2, 0), Rotation2d.fromDegrees(0), 2, 0.15, 3),
            robot.swerve.strafe_line(new Translation2d(4, 0), Rotation2d.fromDegrees(0), 3, 0.05, 0)
        ), robot.swerve);
    }

    public static Pair<Command, Pose2d> app_test() {
        // var auto = dave_auto.from(Path.of("sept26.json"));
        // return Pair.of(auto.rest(), auto.get_starting_pose());
        return Pair.of(Commands.none(), new Pose2d(0, 0, new Rotation2d()));
    }

    public static Command Bill(robot robot) {
        return no_vision(
          Commands.sequence(
            robot.swerve.cmd_reset_pose(new Pose2d(0,0, Rotation2d.fromDegrees(0))),
            robot.swerve.strafe_to_point(new Translation2d(1.73,0.19), 2, 0.05, true),
            robot.swerve.strafe_to_point(new Translation2d(1.82,1.47), 2, 0.05, true),
            robot.swerve.strafe_to_point(new Translation2d(0.50, 1.50 ), 2, 0.06, true),
            robot.swerve.strafe_line(new Translation2d(1.79,0.25), Rotation2d.fromDegrees(90), 0.06, 0),
            robot.swerve.strafe_line(new Translation2d(-0.10, -0.03), Rotation2d.fromDegrees(90), 0.06, 0)
          ), robot.swerve);
    }
}
