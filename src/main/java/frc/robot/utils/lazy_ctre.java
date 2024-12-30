package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

public final class lazy_ctre {
    public static void lazy_config(TalonFX talon, TalonFXConfiguration config) {
        var curr_config = new TalonFXConfiguration();
        talon.getConfigurator().refresh(curr_config);
        if(!ctre_config_equality.is_equal(curr_config, config)) {
            talon.getConfigurator().apply(config);
        }
    }

    public static void lazy_config(Pigeon2 pigeon, Pigeon2Configuration config) {
        var curr_config = new Pigeon2Configuration();
        pigeon.getConfigurator().refresh(curr_config);
        if(!ctre_config_equality.is_equal(curr_config, config)) {
            pigeon.getConfigurator().apply(config);
        }
    }

    public static void lazy_config(CANcoder cancoder, CANcoderConfiguration config) {
        var curr_config = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(curr_config);
        if(!ctre_config_equality.is_equal(curr_config, config)) {
            cancoder.getConfigurator().apply(config);
        }
    }
}
