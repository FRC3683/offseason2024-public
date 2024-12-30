package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class field_util {

    private static Translation2d red_adjust = new Translation2d(0, 0); // new Translation2d(0.5, 0); // kettering
    private static Translation2d blu_adjust = new Translation2d(0, 0); // new Translation2d(-0.6, 0); // kettering

    public static final Translation2d field_size_m = new Translation2d(16.54, 8.21);

    public static Translation2d flip(Translation2d trans) {
        return field_symmetry.flip(trans);
    }

    public static Rotation2d flip(Rotation2d angle) {
        return field_symmetry.flip(angle);
    }
    public static double flip(double angle_rad) {
        return flip(Rotation2d.fromRadians(angle_rad)).getRadians();
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    public static Translation2d flip_dir(Translation2d dir) {
        return field_symmetry.flip_dir(dir);
    }

    public static Translation2d fix(Translation2d blue_trans, Alliance alliance) {
        if(alliance == Alliance.Red) {
            return flip(blue_trans).plus(red_adjust);
        }
        return blue_trans.plus(blu_adjust);
    }

    public static Translation2d fix_raw(Translation2d blue_trans, Alliance alliance) {
        if(alliance == Alliance.Red) {
            return flip(blue_trans);
        }
        return blue_trans;
    }

    public static Rotation2d fix(Rotation2d blue_angle, Alliance alliance) {
        if(alliance == Alliance.Red) {
            return flip(blue_angle);
        }
        return blue_angle;
    }

    public static double fix(double blue_angle_rad, Alliance alliance) {
        return fix(Rotation2d.fromRadians(blue_angle_rad), alliance).getRadians();
    }

    public static Pose2d fix(Pose2d blue_pose, Alliance alliance) {
        if(alliance == Alliance.Red) {
            return flip(blue_pose).plus(new Transform2d(red_adjust, Rotation2d.fromDegrees(0)));
        }
        return blue_pose.plus(new Transform2d(blu_adjust, Rotation2d.fromDegrees(0)));
    }

    public static Pose2d fix_raw(Pose2d blue_pose, Alliance alliance) {
        if(alliance == Alliance.Red) {
            return flip(blue_pose);
        }
        return blue_pose;
    }

    public static Translation2d fix_dir(Translation2d blue_dir, Alliance alliance) {
        if(alliance == Alliance.Red) {
            return flip_dir(blue_dir);
        }
        return blue_dir;
    }

    public static Translation2d fix(Translation2d blue_trans) {
        return fix(blue_trans, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static Translation2d fix_raw(Translation2d blue_trans) {
        return fix_raw(blue_trans, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static Rotation2d fix(Rotation2d blue_angle) {
        return fix(blue_angle, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static double fix(double blue_angle_rad) {
        return fix(Rotation2d.fromRadians(blue_angle_rad)).getRadians();
    }

    public static Pose2d fix(Pose2d blue_pose) {
        return fix(blue_pose, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static Pose2d fix_raw(Pose2d blue_pose) {
        return fix_raw(blue_pose, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public static Translation2d fix_dir(Translation2d blue_dir) {
        return fix_dir(blue_dir, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    private static interface symmetry_type {
        Translation2d flip(Translation2d trans);
        Rotation2d flip(Rotation2d angle);
        Translation2d flip_dir(Translation2d dir);
    }

    @SuppressWarnings("unused")
    private static symmetry_type rotational = new symmetry_type() {
        @Override
        public Translation2d flip(Translation2d trans) {
            return field_size_m.minus(trans);
        }

        @Override
        public Rotation2d flip(Rotation2d angle) {
            return Rotation2d.fromDegrees(180).plus(angle);
        }

        @Override
        public Translation2d flip_dir(Translation2d dir) {
            return dir.times(-1);
        }
    };

    private static symmetry_type mirror = new symmetry_type() {
        @Override
        public Translation2d flip(Translation2d trans) {
            return new Translation2d(field_size_m.getX() - trans.getX(), trans.getY());
        }

        @Override
        public Rotation2d flip(Rotation2d angle) {
            return Rotation2d.fromDegrees(180).minus(angle);
        }

        @Override
        public Translation2d flip_dir(Translation2d dir) {
            return new Translation2d(dir.getX() * -1, dir.getY());
        }


    };

    public static final symmetry_type field_symmetry = mirror;
}
