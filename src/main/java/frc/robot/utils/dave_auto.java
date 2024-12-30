package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.function.Consumer;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot;

public class dave_auto {

    public static final String version = "1.0.0";
    public static final Map<String, Consumer<double[]>> events = new HashMap<>();
    private static final Map<String, Boolean> event_map = new HashMap<>();
    private static swerve swerve;

    private final Queue<Command> commands;
    private final Pose2d starting_pose;

    private dave_auto(dave_path json, boolean reset_odometry) {
        if(!json.version.equals(version)) {
            DriverStation.reportWarning("versions don't match", false);
        }

        starting_pose = reset_odometry ? field_util.fix(new Pose2d(json.start_pos.position.t(), Rotation2d.fromDegrees(json.start_pos.angle_degrees)))
            : null;

        commands = new LinkedList<>();
        for(var waypoint : json.waypoints) {
            boolean is_last = (waypoint == json.waypoints[json.waypoints.length - 1]);
            Command to_point = waypoint.build(is_last);
            commands.add(to_point);
        }
    }

    public Pose2d get_starting_pose() {
        return starting_pose;
    }

    private dave_auto(dave_path json) { this(json, true); }

    public Command next(int num) {
        var command = Commands.none();
        while(!commands.isEmpty() && num > 0) {
            --num;
            command = command.andThen(commands.poll());
        }
        return command;
    }

    public Command next() {
        return next(1);
    }

    public Command rest() {
        return next(commands.size());
    }

    // path relative to deploy directory
    public static dave_auto from(Path path) {
        try {
            var str = Files.readString(Filesystem.getDeployDirectory().toPath().resolve(path));
            return dave_auto.from(str);
        } catch(IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    public static dave_auto from(String json) {
        var object_mapper = new ObjectMapper();
        try {
            var data = object_mapper.readValue(json, dave_path.class);
            var auto = dave_auto.from(data);
            return auto;
        } catch(JsonMappingException e) {
            e.printStackTrace();
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
        return new dave_auto(new dave_path());
    }

    public static dave_auto from(dave_path json) {
        return new dave_auto(json);
    }

    public static void config(swerve swerve) {
        dave_auto.swerve = swerve;
    }

    public static void add_event(String name, Consumer<double[]> event) {
        events.put(name, event);
    }

    public static void add_event(String name, Runnable runnable) {
        add_event(name, (double[] unused) -> { runnable.run(); });
    }

    public static void add_event(String name, Command to_schedule) {
        add_event(name, () -> { to_schedule.schedule(); });
    }

    private static boolean flip() {
        return robot.is_red();
    }


    public static class project_data {
        public String version;
        public dave_point[] named_points;
    }

    static class dave_point {
        public double x, y;
        public String link;

        Translation2d t() {
            return new Translation2d(x, y);
        }

        void flip() {
            var flipped = field_util.flip(t());
            this.x = flipped.getX();
            this.y = flipped.getY();
        }

        @Override
        public String toString() {
            return "(" + x + ", " + y + ": " + link + ")";
        }
    }

    static class dave_event {
        public String event;
        public double[] params;
        public double distance; // the distance from the target point to run the event
    }

    private static class dave_waypoint {
        public String name;
        public String type; // "line", "arc", "point", "assisted"
        public dave_point point;
        public boolean through;
        public double tolerance;
        public double max_vel;
        public double line_angle_degrees;
        public dave_point arc_center;
        public String arc_direction;
        // String theta_type; // "heading", "point", "none"
        // double snap_heading_deg;
        // auto_point_json snap_point;
        public dave_event[] events;

        private Command theta() {
            // switch(theta_type) {
            //     case "heading":     return swerve.snap(snap_heading_deg);
            //     case "point":       return swerve.snap_trans(snap_point.t(), snap_heading_deg);
            //     case "none":        return Commands.none();
            // }
            return Commands.none();
        }

        private Command events() {
            var cmd = Commands.none();
            if(events == null) {
                return cmd;
            }
            for(var event : events) {
                cmd = cmd.alongWith(
                    Commands.waitUntil(() -> swerve.get_pose().getTranslation().minus(point.t()).getNorm() < event.distance)
                    .andThen(() -> { dave_auto.events.get(event.event).accept(event.params); })
                );
            }
            return cmd;
        }

        private Command strafe() {
            switch(type) {
            case "line":
            case "assisted": // TODO
                return swerve.strafe_line(point.t(), Rotation2d.fromDegrees(line_angle_degrees), max_vel, tolerance, 0);
            case "arc":
                return Commands.none();
            case "point":
            default:
                return swerve.strafe_to_point(point.t(), max_vel, tolerance, through);
            }
        }

        private Command build(boolean last) {
            if(last) {
                tolerance = -1;
            }
            if(flip()) {
                point.flip();
                // if(snap_point != null) {
                //     snap_point.flip();
                // }
                // if(!theta_type.equals("point")) {
                //     snap_heading_deg = field_util.fix(snap_heading_deg);
                // }
            }
            return Commands.parallel(
                strafe(),
                events(),
                Commands.runOnce(() -> {
                    theta().schedule();
                })
            );
        }
    }

    private static class start_position {
        public dave_point position;
        public double angle_degrees;
    }

    private static class dave_path {
        public String version = "";
        public start_position start_pos = new start_position();
        public dave_waypoint[] waypoints = {};
    }

    public static interface swerve {
        Command strafe_to_point(final Translation2d target, final double max_vel, final double tolerance, final boolean through);
        Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance, final double through_vel);
        Command strafe_assisted(Translation2d target, Rotation2d direction, double max_vel, double tolerance, final boolean through);
        Command snap(double theta_deg);
        Command snap_trans(Translation2d look_at, double offset_deg);
        Pose2d get_pose();
        void reset_pose(Pose2d pose);
    }
}
