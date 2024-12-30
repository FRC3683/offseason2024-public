package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;
import frc.robot.subsystems.swerve;
import frc.robot.utils.controls.swerve_module;

// 
// BEVELS TO THE LEFT WHEN RUN
// 

public class swerve_setup_offsets extends Command {
    private final static int total_samples = 2 * constants.control_freq;

    private swerve_setup_offsets(swerve_module module) {
        this.module = module;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        integral = 0;
        samples = 0;
    }

    @Override
    public void execute() {
        ++samples;
        integral += module.get_abs_raw();
    }

    @Override
    public boolean isFinished() {
        return samples >= total_samples;
    }

    @Override
    public void end(boolean interrupted) {
        double average = integral / (double)samples;
        module.save_abs_offset(average);
        module.reset_turning();
        System.out.println(module.nt_name + " absolute offsets saved");
    }

    private swerve_module module;
    private double integral;
    private int samples;

    public static Command make(swerve_module[] modules) {
        Command commands = Commands.none();
        for(var module : modules) {
            commands = commands.alongWith(new swerve_setup_offsets(module));
        }
        return Commands.print(
            "started getting swerve absolute offsets, please wait\n"
            + "did u align the wheels with the bevel gears to the left? you should have. before running this."
        ).andThen(commands).ignoringDisable(true);
    }
}
