package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Add your docs here.
 */
public class math_utils {
    public static double average(double a, double b) {
        return (a + b) / 2;
    }

    public static double sq(double x) {
        return x * x;
    }

    public static double cube(double x) {
        return x * x * x;
    }

    public static double qu(double x) {
        return x * x * x * x;
    }

    public static double quint(double x) {
        return x * x * x * x * x;
    }

    public static double lerp(double start, double end, double alpha) {
        return start * (1.0 - alpha) + end * alpha;
    }

    public static double unlerp(double start, double end, double value) {
        return (value - start) / (end - start);
    }

    public static double remap(double old_start, double old_end, double old_value, double new_start, double new_end) {
        return lerp(new_start, new_end, unlerp(old_start, old_end, old_value));
    }

    public static double remap_clamp(double old_start, double old_end, double old_value, double new_start, double new_end) {
        return clamp(Math.min(new_start, new_end), Math.max(new_start, new_end), remap(old_start, old_end, old_value, new_start, new_end));
    }

    public static Translation2d clamp(Translation2d t, double max_length) {
        double len = t.getNorm();
        if(len < max_length) {
            return t;
        }
        return t.div(len).times(max_length);
    }

    public static Rotation2d remap(Rotation2d old_start, Rotation2d old_end, Rotation2d old_value, Rotation2d new_start, Rotation2d new_end) {
        return Rotation2d.fromRadians(
            remap(old_start.getRadians(), old_end.getRadians(), 
            old_value.getRadians(),
            new_start.getRadians(), new_end.getRadians())
        );
    }

    public static double hypot(double a, double b) {
        return Math.sqrt((a * a) + (b * b));
    }

    public static double hypot(ChassisSpeeds speeds) {
        return hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public static double cubic_bezier(double start, double start_q, double end, double end_q, double alpha) {
        double a = start * cube(1.0 - alpha);
        double b = 3 * start_q * sq(1.0 - alpha) * alpha;
        double c = 3 * end_q * (1.0 - alpha) * sq(alpha);
        double d = end * cube(alpha);
        return a + b + c + d;
    }

    public static double clamp(double min, double max, double value) {
        return value < min ? min : (value > max ? max : value);
    }
    public static int clampi(int min, int max, int value) {
        return value < min ? min : (value > max ? max : value);
    }

    public static double clamp01(double value) {
        return clamp(0.0, 1.0, value);
    }

    public static double clamp1(double value) {
        return clamp(-1.0, 1.0, value);
    }

    public static double clampN(double n, double value) {
        return clamp(-n, n, value);
    }

    public static double clamp0N(double max, double value) {
        return clamp(0.0, max, value);
    }

    public static double squared_input(double x) {
        return x > 0 ? sq(x) : -sq(x);
    }

    public static boolean close_enough(double num1, double num2, double error) {
        return Math.abs(num1 - num2) <= error;
    }
    public static boolean close_enough(Rotation2d rot1, Rotation2d rot2, Rotation2d err) {
        return close_enough(rot1.getRadians(), rot2.getRadians(), err.getRadians());
    }

    public static boolean close_enough_percent(double num1, double num2, double alpha) {
        return Math.abs(num1 - num2) <= Math.abs(num2 * alpha);
    }

    public static boolean close_enough(Translation2d p1, Translation2d p2, double dist) {
        var diff = p2.minus(p1);
        return sq(diff.getX()) + sq(diff.getY()) < sq(dist);
    }

    private static double[] EMPTY_DOUBLE_ARRAY = {};

    public static double[] toPrimitive(Double[] array) {
        if (array == null) {
            return null;
        } else if (array.length == 0) {
            return EMPTY_DOUBLE_ARRAY;
        }
        final double[] result = new double[array.length];
        for (int i = 0; i < array.length; i++) {
            result[i] = array[i].doubleValue();
        }
        return result;
    }

    public static boolean within_circle(double dx, double dy, double r) {
        return (dx * dx + dy * dy) < (r * r);
    }

    public static boolean slower_than(ChassisSpeeds speeds, double mps) {
        return sq(speeds.vxMetersPerSecond) + sq(speeds.vyMetersPerSecond) <= sq(mps);
    }

    public static Translation2d lerp(Translation2d from, Translation2d to, double percent) {
        return from.plus(to.minus(from).times(percent));
    }

    public static Rotation2d clamp(Rotation2d min ,Rotation2d max, Rotation2d value) {
        return Rotation2d.fromRadians(clamp(min.getRadians(), max.getRadians(), value.getRadians()));
    }

    public static Translation2d almost(Translation2d from, Translation2d to, double back) {
        var diff = to.minus(from);
        return from.plus(diff.times(1 - (back / diff.getNorm())));
    }

    /**
     * Grand theft poofs code
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static Pose2d exp(final Twist2d delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = 0.5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < 1E-9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    public static double closest90deg(double deg) {
        return Math.round(deg / 90.0) * 90.0;
    }

    public static double closest90rad(double rad) {
        return Math.round(rad / (Math.PI / 2)) * (Math.PI / 2);
    }

    public static Rotation2d abs(Rotation2d rot) {
        return Rotation2d.fromRadians(Math.abs(rot.getRadians()));
    }

    public static Translation2d find_translation(Rotation2d theta, double hypotenoose_meters) {
        return new Translation2d(theta.getCos() * hypotenoose_meters, theta.getSin() * hypotenoose_meters);
    }

    public static double dot(double ax, double ay, double bx, double by) {
        return (ax * bx) + (ay * by);
    }

    public static double dot(Translation2d a, Translation2d b) {
        return dot(a.getX(), a.getY(), b.getX(), b.getY());
    }

    public static Translation2d project(Translation2d from, Translation2d onto) {
        Translation2d onto_hat = onto.div(onto.getNorm());
        return onto_hat.times(dot(from, onto_hat));
    }

    public static Rotation2d angle(Translation2d from, Translation2d to) {
        return Rotation2d.fromRadians(Math.acos(dot(from, to) / from.getNorm() / to.getNorm()));
    }

    public static Rotation2d err(Rotation2d target, Rotation2d curr, boolean clockwise) {
        return target.minus(curr);
    }

    public static Pose2d plus(Pose2d pose, Translation2d trans) {
        return pose.plus(new Transform2d(trans.getX(), trans.getY(), Rotation2d.fromRadians(0)));
    }

    public static Translation2d trans(ChassisSpeeds speeds) {
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    // return unit vector with direction of vec
    public static Translation2d hat(Translation2d vec) {
        final var length = vec.getNorm();
        return new Translation2d(vec.getX() / length, vec.getY() / length);
    }

    public static Color8Bit add(Color8Bit c0, Color8Bit c1) {
        return new Color8Bit(c0.red + c1.red, c0.green + c1.green, c0.blue + c1.blue);
    }

    public static Color8Bit mult(Color8Bit c, double v) {
        return new Color8Bit((int) Math.round(c.red * v), (int) Math.round(c.green * v), (int) Math.round(c.blue * v));
    }

    public static double move_toward(double curr, double target, double max_delta) {
        if(close_enough(curr, target, max_delta)) {
            return target;
        }
        return curr + clamp(-max_delta, max_delta, target - curr);
    }

    public static double hypot_squred(Translation2d t) {
        return t.getX() * t.getX() + (t.getY() * t.getY());
    }

    public static double hypot_squred(double dx, double dy) {
        return dx * dx + (dy * dy);
    }

    public static double hypot_squred(ChassisSpeeds s1, ChassisSpeeds s2) {
        double dx = s1.vxMetersPerSecond - s2.vxMetersPerSecond;
        double dy = s1.vyMetersPerSecond - s2.vyMetersPerSecond;
        return (dx * dx) + (dy * dy);
    }

    public static boolean within(Translation2d t1, Translation2d t2, double d) {
        return hypot_squred(t1.minus(t2)) <= d;
    }

    public static void move_toward(ChassisSpeeds input, ChassisSpeeds to, double max_delta) {
        double dx = to.vxMetersPerSecond - input.vxMetersPerSecond;
        double dy = to.vyMetersPerSecond - input.vyMetersPerSecond;
        double delta = hypot(dx, dy);
        if(delta <= max_delta) {
            input.vxMetersPerSecond = to.vxMetersPerSecond;
            input.vyMetersPerSecond = to.vyMetersPerSecond;
            return;
        }
        input.vxMetersPerSecond += (dx / delta) * max_delta;
        input.vyMetersPerSecond += (dy / delta) * max_delta;
    }

    // public static Translation2d move_toward(Translation2d from, Translation2d to, double max_diff) throws Exception {
    //     if(within(from, to, max_diff)) {
    //         return to;
    //     }
    // }
    
}