package frc.robot.utils;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;

public class n_point_remap {

    private final Point[] points;

    public final double min_x, min_y, max_x, max_y;

    public n_point_remap(double... coords) {
        if(coords.length % 2 != 0) {
            DriverStation.reportError("should have an x for every y pls", false);
        }

        points = new Point[coords.length / 2];
        if(points.length < 2) {
            DriverStation.reportError("need 2 points at least", false);
        }

        for(int i = 0; i < points.length; ++i) {
            points[i] = new Point(coords[i * 2], coords[i * 2 + 1]);
        }

        Arrays.sort(points);

        min_x = get_min().x;
        min_y = get_min().y;
        max_x = get_max().x;
        max_y = get_max().y;
    }

    public Point get_min() {
        return points[0];
    }

    public Point get_max() {
        return points[points.length - 1];
    }

    public double calc(double x) {
        if(points.length == 2) {
            return math_utils.remap_clamp(min_x, max_x, x, min_y, max_y);
        }

        if(x < min_x) {
            return min_y;
        }
        for(int i = 1; i < points.length; ++i) {
            Point upper = points[i];
            if(x < upper.x) {
                Point lower = points[i-1];
                return math_utils.remap(lower.x, upper.x, x, lower.y, upper.y);
            }
        }
        return max_y;
    }

    private static class Point implements Comparable<Point> {
        public final double x, y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public int compareTo(Point other) {
            return Double.compare(x, other.x);
        }
    }
}
