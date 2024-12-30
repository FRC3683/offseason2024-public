package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve;
import frc.robot.utils.controls.swerve_kin2.module_state;

public final class drive_goofy {
    

    public static Command drive_rc_car(swerve swerve, Supplier<Double> speedy, Supplier<Double> turny, double max_speed, double max_turn_radps) {
        var states = new module_state[4];
        for(int i = 0; i < states.length; ++i) {
            states[i] = new module_state(0, 0, 0);
        }
        return swerve.strafe_omega(() -> {
            final var speed01 = speedy.get();
            final var speed = speed01 * max_speed;
            final var turn = turny.get() * max_turn_radps;
            return new ChassisSpeeds(speed, 0, turn * speed01);
        });
    }
}
