package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface configurable {
    void configure();

    public static void configure_all(configurable... configurables) {
        configure_all(Commands.none(), configurables);
    }

    public static void configure_all(Command along_with, configurable... configurables) {
        if(RobotBase.isSimulation()) {
            return;
        }
        var cmd = Commands.print("configuring in 1s").andThen(Commands.waitSeconds(1)).andThen(Commands.print("configuring..."));
        for(var con : configurables) {
            cmd = cmd.andThen(Commands.runOnce(() -> { con.configure(); })).andThen(Commands.waitSeconds(0.06));
        }
        cmd.deadlineWith(along_with).ignoringDisable(true).schedule();
    }
}
