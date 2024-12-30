package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;

public abstract class dave_led {

    public static final Color8Bit clr_off = new Color8Bit(0, 0, 0);
    public static final Color8Bit clr_cyan = new Color8Bit(0, 255, 255); // cyan
    public static final Color8Bit clr_white = new Color8Bit(255, 255, 255);
    public static final Color8Bit clr_green = new Color8Bit(0, 150, 0);
    public static final Color8Bit clr_red = new Color8Bit(150, 0, 0);
    public static final Color8Bit clr_blue = new Color8Bit(0, 0, 255);
    public static final Color8Bit clr_purple = new Color8Bit(255, 0, 255);
    public static final Color8Bit clr_oramge = new Color8Bit(255, 50, 0);
    public static final Color8Bit clr_yellow = new Color8Bit(255, 255, 0);

    private AddressableLED addressable_led;
    private AddressableLEDBuffer led_buffer;
    private double rainbow_first_pixel_hue = 0;
    private final Timer timer = new Timer();
    protected final int length;

    public dave_led(int port, int length, TimedRobot robot) {
        this.length = length;
        led_buffer = new AddressableLEDBuffer(length);
        addressable_led = new AddressableLED(port);
        addressable_led.setLength(led_buffer.getLength());
        addressable_led.setData(led_buffer);
        addressable_led.start();
        timer.start();

        robot.addPeriodic(this::lowlevel_periodic, constants.control_dts);
    }

    protected abstract void periodic();

    private void lowlevel_periodic() {
        periodic();
        addressable_led.setData(led_buffer);
    }

    public class flag {
        private int value = 0;

        public void increment() {
            ++value;
        }

        public void decrement() {
            --value;
        }

        public boolean get() {
            return value != 0;
        }

        public Command run() {
            return Commands.startEnd(() -> {
                increment();
            }, () -> {
                decrement();
            }).ignoringDisable(true);
        }
    }

    protected void set_fast_flicker(Color8Bit color) {
        if(Math.round(timer.get() * 20) % 2 == 0) {
            set_color(color);
        } else {
            set_color(0, length, clr_off);
        }
    }

    protected void set_slow_flash(Color8Bit color) {
        if(Math.round(timer.get() * 10) % 2 == 0) {
            set_color(color);
        } else {
            set_color(0, length, math_utils.mult(color, 0.1));
        }
    }

    protected void charge(BooleanSupplier... happy_sups) {
        // hsv hue 0 = red, 50 = green
        int width = length / happy_sups.length;
        for(int i = 0; i < happy_sups.length; ++i) {
            var sup = happy_sups[happy_sups.length - i - 1];
            var percent = sup.getAsBoolean();
            set_color(i * width, (i+1) * width, percent ? clr_green : clr_red);
        }
    }

    protected void set_color(int start, int end, Color8Bit color) {
        for (var i = Math.max(0, start); i < Math.min(length, end); ++i) {
            led_buffer.setRGB(i, color.red, color.green, color.blue);
        }
    }

    protected void set_checker(int start, int end, Color8Bit c0, Color8Bit c1) {
        boolean alternate = Math.round(timer.get() * 6) % 2 == 0;
        for (var i = Math.max(0, start); i < Math.min(length, end); ++i) {
            if ((i % 2 == 0) ^ alternate) {
                led_buffer.setRGB(i, c0.red, c0.green, c0.blue);
            } else {
                led_buffer.setRGB(i, c1.red, c1.green, c1.blue);
            }
        }
    }

    protected void set_single(int index, Color8Bit c) {
        if(index < 0 || index >= length) {
            return;
        }
        led_buffer.setRGB(index, c.red, c.green, c.blue);
    }

    protected void add_single(int index, Color8Bit c) {
        if(index < 0 || index >= length) {
            return;
        }
        set_single(index, math_utils.add(new Color8Bit(led_buffer.getLED(index)), c));
    }

    protected void set_trail(Color8Bit color) {
        int index = (int)Math.round(Math.sin(timer.get() * 3) * 7) + 7;
        add_single(index - 1, math_utils.mult(color, 0.02));
        add_single(index, color);
        add_single(index + 1, math_utils.mult(color, 0.02));
    }

    protected void set_color(Color8Bit c) {
        set_color(0, length, c);
    }

    protected void rainbow(int value) {
        for (var i = 0; i < length; i++) {
            final int hue = (int)(rainbow_first_pixel_hue + (i * 60 / length)) % 180;
            led_buffer.setHSV(i, hue, 255, value);
        }
        rainbow_first_pixel_hue += 50 * constants.control_dts;
        rainbow_first_pixel_hue %= 180;
    }
}