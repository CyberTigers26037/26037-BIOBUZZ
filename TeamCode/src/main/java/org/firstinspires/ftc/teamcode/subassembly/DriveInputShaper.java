package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DriveInputShaper {
    // ---- Tunables ----
    public static double DEADBAND = 0.05;        // ignore stick values smaller than this
    public static double EXPO = 0.6;             // 0 = linear, 1 = fully cubic
    public static double ACCEL_RATE = 3.0;       // max power change per second when speeding up (0 -> full in ~0.33 s)
    public static double DECEL_RATE = 6.0;       // max power change per second when slowing down (faster, so stopping feels crisp)
    public static double TURN_ACCEL_RATE = 4.0;  // turning can usually ramp a bit faster
    public static double SLOW_MODE_SCALE = 0.4;  // 40% power in slow mode

    private final ElapsedTime timer = new ElapsedTime();
    private double forward, strafe, turn;

    public void update(double rawForward, double rawStrafe, double rawTurn, boolean slowMode) {
        double dt = Math.min(timer.seconds(), 0.1);  // ignore long pauses (e.g. first loop)
        timer.reset();

        double scale = slowMode ? SLOW_MODE_SCALE : 1.0;

        forward = slew(forward, curve(rawForward) * scale, ACCEL_RATE, dt);
        strafe  = slew(strafe,  curve(rawStrafe)  * scale, ACCEL_RATE, dt);
        turn    = slew(turn,    curve(rawTurn)    * scale, TURN_ACCEL_RATE, dt);
    }

    /** Deadband plus a blend of linear and cubic response. */
    private static double curve(double x) {
        if (Math.abs(x) < DEADBAND) return 0;
        double sign = Math.signum(x);
        // Rescale so output starts at 0 just past the deadband (no sudden jump)
        double m = (Math.abs(x) - DEADBAND) / (1.0 - DEADBAND);
        return sign * ((1 - EXPO) * m + EXPO * m * m * m);
    }

    /** Move 'current' toward 'target', no faster than the allowed rate. */
    private static double slew(double current, double target, double accelRate, double dt) {
        // Reversing direction? Come down to zero first, then ramp up the other way.
        if (current * target < 0) target = 0;

        double rate = Math.abs(target) < Math.abs(current) ? DECEL_RATE : accelRate;
        double maxStep = rate * dt;
        return current + Range.clip(target - current, -maxStep, maxStep);
    }

    public void reset() { forward = strafe = turn = 0; timer.reset(); }

    public double getForward() { return forward; }
    public double getStrafe()  { return strafe; }
    public double getTurn()    { return turn; }
}
