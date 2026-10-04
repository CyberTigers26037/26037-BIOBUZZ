package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.pedropathing.drivetrain.Drivetrain;

import java.util.Map;

public class DrivetrainHelper {
    public static String debugString(Drivetrain drivetrain) {
        // Some of the tuners call a method debugString() on the Drivetrain
        // interface, but this method does not exist.
        // When this method exists at some point in the future,
        // this helper class and method should be deleted...
        StringBuilder output = new StringBuilder();

        output.append("Drivetrain: {");
        Map<String, Object> debug = drivetrain.debug();
        debug.forEach((k, v) -> output.append("\n").append(k).append(" : ").append(v));
        output.append("\n}");

        return output.toString();
    }
}
