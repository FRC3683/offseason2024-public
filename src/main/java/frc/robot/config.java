package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class config {

    public static final boolean dev = true; // extra networktable values for code stuff

    public static final String can_ivore = "canivore";
    public static final String can_rio = "rio";

    public static final String drive_canbus = can_ivore;
    public static final String rest_canbus = can_ivore;
    
    public static final int can_swerve_fl_turn = 3;
    public static final int can_swerve_fr_turn = 9;
    public static final int can_swerve_bl_turn = 5;
    public static final int can_swerve_br_turn = 4;
    
    public static final int can_swerve_fl_drive = 6;
    public static final int can_swerve_fr_drive = 8;
    public static final int can_swerve_bl_drive = 7;
    public static final int can_swerve_br_drive = 2;

    public static final int dio_swerve_fl_abs = 8;
    public static final int dio_swerve_fr_abs = 6;
    public static final int dio_swerve_bl_abs = 0;
    public static final int dio_swerve_br_abs = 7;
    
    public static final int can_pigeon = 10;

    public static final int[] can_loop = { can_pigeon, can_swerve_fl_turn, can_swerve_fl_drive, 
        can_swerve_fr_turn, can_swerve_fr_drive, 
        can_swerve_bl_drive, can_swerve_bl_turn,
        can_swerve_br_turn, can_swerve_br_drive };

    public static final int pwm_leds = 4;


    public static final String LL_shooter = "limelight-shooter";
    public static final String LL_intake = "limelight-intake";
    public static final LL intake_mount = new LL(new Translation3d(0.25, 0.093, 0.44), Rotation2d.fromDegrees(55));
    public static final LL shooter_mount = new LL(new Translation3d(0.225, 0.093, 0.672), Rotation2d.fromDegrees(13.736)); // yaw 174.5
    // shooter crosshair x: -0.1957, y: 0
    // intake crosshair x: 0.021875, y: -1
    public static final int[] valid_ids = { 4, 5, 6, 7, 11, 12, 13, 14, 15, 16 }; // 8 and 3 are the wobbly warped ones, 1, 2, 9, 10 are HP loading, bad for auto



    public final class swerve {

        public static final boolean pro = true;

        private static final double drive_kS = 0.22;
        private static final double max_speed_rotations_ps = Units.radiansToRotations(constants.swerve.max_module_speed_mps / constants.swerve.wheel_radius);
        private static final double drive_kV = (12.0 - drive_kS) / max_speed_rotations_ps;

        public static TalonFXConfiguration drive_configs(InvertedValue inversion) {
            return new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(50)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(50)
                    .withSupplyCurrentLimitEnable(true)
                )
                .withSlot0(new Slot0Configs()
                    .withKV(drive_kV)
                    .withKP(2.0) // TODO tune more... maybe idk
                    .withKS(drive_kS)
                    .withKD(0.0)
                    .withKI(0.0)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.01)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.swerve.module.drive_ratio)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Brake)
                )
            ;
        }

        public static TalonFXConfiguration turn_configs(InvertedValue inversion) {
            var closed_loop = new ClosedLoopGeneralConfigs();
            closed_loop.ContinuousWrap = true;
            return new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.swerve.module.steer_ratio)
                )
                .withSlot0(new Slot0Configs()
                    .withKV(3.0) // TODO tune
                    .withKP(90)
                    .withKD(0.0)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.02)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Brake)
                )
                .withClosedLoopGeneral(closed_loop)
            ;
        }

        public static final Slot0Configs sim_turn_configs = new Slot0Configs()
            .withKV(3.0).withKP(90.0).withKD(0.0)
        ;
        public static final Slot0Configs sim_drive_configs = new Slot0Configs()
            .withKS(drive_kS).withKV(drive_kV).withKP(2.0)
        ;

        public static final class module_config {
            public final String name;
            public final int can_drive, can_turn, dio_abs;
            public final InvertedValue drive_inverted;
            public final InvertedValue turn_inverted;
            public final double comp_abs_offset, prac_abs_offset;

            public module_config(String name, int can_drive, int can_turn, int dio_abs, 
                    InvertedValue drive_inverted, InvertedValue turn_inverted, 
                    double comp_abs_offset, double prac_abs_offset) {
                this.name = name;
                this.can_drive = can_drive;
                this.can_turn = can_turn;
                this.dio_abs = dio_abs;
                this.drive_inverted = drive_inverted;
                this.turn_inverted = turn_inverted;
                this.comp_abs_offset = comp_abs_offset;
                this.prac_abs_offset = prac_abs_offset;
            }
        }

        public static final module_config[] module_configs = {
            new module_config("fr", can_swerve_fr_drive, can_swerve_fr_turn, dio_swerve_fr_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                1.236, 0), // TODO

            new module_config("fl", can_swerve_fl_drive, can_swerve_fl_turn, dio_swerve_fl_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                0.832, 0),

            new module_config("br", can_swerve_br_drive, can_swerve_br_turn, dio_swerve_br_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                4.325, 0),

            new module_config("bl", can_swerve_bl_drive, can_swerve_bl_turn, dio_swerve_bl_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive,
                5.339, 0),
        };
    }

    public final class leds {
        public static final int length = 17;
    }

    public static final class LL {
        public final Translation3d mount_offset; // from center of robot, height from ground
        public final Rotation2d mount_angle; // from vertical to lense normal

        public LL(Translation3d mount_offset, Rotation2d mount_angle) {
            this.mount_offset = mount_offset;
            this.mount_angle = mount_angle;
        }
    }
}
