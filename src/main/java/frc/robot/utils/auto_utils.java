package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve;

/*
 * FOR EASE OF USE:
 * import static frc.robot.utils.auto_utils.*;
 */
public class auto_utils {

    /*
     * begin a set of commands when this command is scheduled, then let them run without caring about cancelling them.
     * use for snap, shots, etc.
     */
    public static Command async(Command... commands) {
        return Commands.runOnce(() -> {
            for(var command : commands) {
                command.schedule();
            }
        });
    }

    /*
     * end the subsystem's currently running command
     */
    public static Command end_current_command(Subsystem subsystem) {
        return Commands.runOnce(() -> {
            subsystem.getCurrentCommand().cancel();
        });
    }

    /*
     * wrap command with reject vision updates, useful for straight line tests and wheel calibration
     */
    public static Command no_vision(Command command, swerve swerve) {
        return swerve.cmd_reject_vision(true)
            .alongWith(command)
            .finallyDo(() -> {
                swerve.reject_vision_pose(false);
            }
        );
    }
}
