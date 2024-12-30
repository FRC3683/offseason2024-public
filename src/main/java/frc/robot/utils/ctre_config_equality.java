package frc.robot.utils;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;


/*
 *  THANKYOU 2023 POOFS
 */
public class ctre_config_equality {

    public static boolean ENABLE_LOGGING_INEQ = false;
    public static final double CONFIG_EPSILON = 0.05;

    //=================================================================|
    //                                                                 |
    //                                TALON FX                         |
    //                                                                 |
    //=================================================================|

    public static boolean is_equal(TalonFXConfiguration a, TalonFXConfiguration b) {
        return is_equal(a.Slot0, b.Slot0) &&
                is_equal(a.Slot1, b.Slot1) &&
                is_equal(a.Slot2, b.Slot2) &&
                is_equal(a.MotorOutput, b.MotorOutput) &&
                is_equal(a.CurrentLimits, b.CurrentLimits) &&
                is_equal(a.Voltage, b.Voltage) &&
                is_equal(a.TorqueCurrent, b.TorqueCurrent) &&
                is_equal(a.Feedback, b.Feedback) &&
                is_equal(a.OpenLoopRamps, b.OpenLoopRamps) &&
                is_equal(a.ClosedLoopRamps, b.ClosedLoopRamps) &&
                is_equal(a.HardwareLimitSwitch, b.HardwareLimitSwitch) &&
                is_equal(a.Audio, b.Audio) &&
                is_equal(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch) &&
                is_equal(a.MotionMagic, b.MotionMagic);
    }

    public static boolean is_equal(Slot0Configs a, Slot0Configs b) {
        boolean val = math_utils.close_enough(a.kP,b.kP,CONFIG_EPSILON) &&
        math_utils.close_enough(a.kI,b.kI,CONFIG_EPSILON) &&
        math_utils.close_enough(a.kD,b.kD,CONFIG_EPSILON) &&
        math_utils.close_enough(a.kV,b.kV,CONFIG_EPSILON) &&
        math_utils.close_enough(a.kS,b.kS,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Slot0Configs not equal");
        }
        return val;
    }

    public static boolean is_equal(Slot1Configs a, Slot1Configs b) {
        boolean val = math_utils.close_enough(a.kP,b.kP,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kI,b.kI,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kD,b.kD,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kV,b.kV,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kS,b.kS,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Slot1Configs not equal");
        }
        return val;
    }

    public static boolean is_equal(Slot2Configs a, Slot2Configs b) {
        boolean val = math_utils.close_enough(a.kP,b.kP,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kI,b.kI,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kD,b.kD,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kV,b.kV,CONFIG_EPSILON) &&
                math_utils.close_enough(a.kS,b.kS,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Slot2Configs not equal");
        }
        return val;
    }

    public static boolean is_equal(MotorOutputConfigs a, MotorOutputConfigs b) {
        boolean val = a.Inverted.value == b.Inverted.value
                && a.NeutralMode.value == b.NeutralMode.value
                && math_utils.close_enough(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband,CONFIG_EPSILON)
                && math_utils.close_enough(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle,CONFIG_EPSILON)
                && math_utils.close_enough(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("MotorOutputConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
        boolean val = math_utils.close_enough(a.StatorCurrentLimit, b.StatorCurrentLimit,CONFIG_EPSILON)
                && math_utils.close_enough(a.SupplyCurrentLimit, b.SupplyCurrentLimit,CONFIG_EPSILON)
                && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
                && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("CurrentLimitsConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(VoltageConfigs a, VoltageConfigs b) {
        boolean val = math_utils.close_enough(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant,CONFIG_EPSILON)
                && math_utils.close_enough(a.PeakForwardVoltage, b.PeakForwardVoltage, 1e-12)
                && math_utils.close_enough(a.PeakReverseVoltage, b.PeakReverseVoltage, 1e-12);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("VoltageConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
        boolean val = math_utils.close_enough(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent,CONFIG_EPSILON)
                && math_utils.close_enough(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent,CONFIG_EPSILON)
                && math_utils.close_enough(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("TorqueCurrentConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(FeedbackConfigs a, FeedbackConfigs b) {
        boolean val = math_utils.close_enough(a.FeedbackRotorOffset, b.FeedbackRotorOffset,CONFIG_EPSILON)
                && math_utils.close_enough(a.SensorToMechanismRatio, b.SensorToMechanismRatio,CONFIG_EPSILON)
                && math_utils.close_enough(a.RotorToSensorRatio, b.RotorToSensorRatio,CONFIG_EPSILON)
                && a.FeedbackSensorSource.value == b.FeedbackSensorSource.value
                && a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("FeedbackConfigs not equal");
            System.out.printf("A-FeedbackRotorOffset: %f, B-FeedbackRotorOffset: %f\n", a.FeedbackRotorOffset, b.FeedbackRotorOffset);
            System.out.printf("A-SensorToMechanismRatio: %f, B-SensorToMechanismRatio: %f\n", a.SensorToMechanismRatio, b.SensorToMechanismRatio);
            System.out.printf("A-RotorToSensorRatio: %f, B-RotorToSensorRatio: %f\n", a.RotorToSensorRatio, b.RotorToSensorRatio);
            System.out.printf("A-FeedbackSensorSource: %d, B-FeedbackSensorSource: %d\n", a.FeedbackSensorSource.value, b.FeedbackSensorSource.value);
            System.out.printf("A-FeedbackRemoteSensorID: %d, B-FeedbackRemoteSensorID: %d\n", a.FeedbackRemoteSensorID, b.FeedbackRemoteSensorID);

        }
        return val;
    }

    public static boolean is_equal(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
        boolean val = math_utils.close_enough(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod,CONFIG_EPSILON)
                && math_utils.close_enough(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod,CONFIG_EPSILON)
                && math_utils.close_enough(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("OpenLoopRampsConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
        boolean val = math_utils.close_enough(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod,CONFIG_EPSILON)
                && math_utils.close_enough(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod,CONFIG_EPSILON)
                && math_utils.close_enough(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("ClosedLoopRampsConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
        boolean val = a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
                && b.ForwardLimitEnable == b.ForwardLimitEnable
                && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
                && a.ReverseLimitEnable == b.ReverseLimitEnable
                && math_utils.close_enough(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue,CONFIG_EPSILON)
                && math_utils.close_enough(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue,CONFIG_EPSILON)
                && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
                && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
                && a.ForwardLimitSource.value == b.ForwardLimitSource.value
                && a.ReverseLimitSource.value == b.ReverseLimitSource.value
                && a.ForwardLimitType.value == b.ForwardLimitType.value
                && a.ReverseLimitType.value == b.ReverseLimitType.value;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("HardwareLimitSwitchConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(AudioConfigs a, AudioConfigs b) {
        boolean val = a.BeepOnBoot == b.BeepOnBoot;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("AudioConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
        boolean val = math_utils.close_enough(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold,CONFIG_EPSILON)
                && math_utils.close_enough(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold,CONFIG_EPSILON)
                && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
                && a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("SoftwareLimitSwitchConfigs not equal");
        }
        return val;
    }

    public static boolean is_equal(MotionMagicConfigs a, MotionMagicConfigs b) {
        boolean val = math_utils.close_enough(a.MotionMagicAcceleration, b.MotionMagicAcceleration,CONFIG_EPSILON)
                && math_utils.close_enough(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity,CONFIG_EPSILON)
                && math_utils.close_enough(a.MotionMagicJerk, b.MotionMagicJerk,CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("MotionMagicConfigs not equal");
        }
        return val;
    }

    //=================================================================|
    //                                                                 |
    //                                PIGEON                           |
    //                                                                 |
    //=================================================================|

    public static boolean is_equal(Pigeon2Configuration a, Pigeon2Configuration b) {
        return is_equal(a.GyroTrim, b.GyroTrim) &&
                is_equal(a.MountPose, b.MountPose) &&
                is_equal(a.Pigeon2Features, b.Pigeon2Features);
    }

    private static boolean is_equal(Pigeon2FeaturesConfigs a, Pigeon2FeaturesConfigs b) {
        boolean val = a.DisableNoMotionCalibration == b.DisableNoMotionCalibration
                && a.DisableTemperatureCompensation == b.DisableTemperatureCompensation
                && a.EnableCompass == b.EnableCompass;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("Pigeon2FeaturesConfigs not equal");
        }
        return val;
    }

    private static boolean is_equal(MountPoseConfigs a, MountPoseConfigs b) {
        boolean val = math_utils.close_enough(a.MountPosePitch, b.MountPosePitch, CONFIG_EPSILON)
                && math_utils.close_enough(a.MountPoseRoll, b.MountPoseRoll, CONFIG_EPSILON)
                && math_utils.close_enough(a.MountPoseYaw, b.MountPoseYaw, CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("MountPoseConfigs not equal");
        }
        return val;
    }

    private static boolean is_equal(GyroTrimConfigs a, GyroTrimConfigs b) {
        boolean val = math_utils.close_enough(a.GyroScalarX, b.GyroScalarX, CONFIG_EPSILON)
                && math_utils.close_enough(a.GyroScalarY, b.GyroScalarY, CONFIG_EPSILON)
                && math_utils.close_enough(a.GyroScalarZ, b.GyroScalarZ, CONFIG_EPSILON);
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("GyroTrimConfigs not equal");
        }
        return val;
    }

    //=================================================================|
    //                                                                 |
    //                                CANCODER                         |
    //                                                                 |
    //=================================================================|

    public static boolean is_equal(CANcoderConfiguration a, CANcoderConfiguration b) {
        return is_equal(a.MagnetSensor, b.MagnetSensor);
    }

    private static boolean is_equal(MagnetSensorConfigs a, MagnetSensorConfigs b) {
        boolean val = math_utils.close_enough(a.MagnetOffset, b.MagnetOffset, CONFIG_EPSILON)
                && a.SensorDirection == b.SensorDirection
                && a.AbsoluteSensorRange == b.AbsoluteSensorRange;
        if (ENABLE_LOGGING_INEQ && !val) {
            System.out.println("MagnetSensorConfigs not equal");
        }
        return val;
    }
}