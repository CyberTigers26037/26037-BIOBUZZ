package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.util.Device;

public class RobotConfig {
    private static boolean initialized;
    private static final String ROBOT_SERIAL_COACH_ANTHONY          = "b59fde0caa12667d";
    private static final String ROBOT_SERIAL_MILLENNIUM_STARTER_BOT = "a542e45d0aef5058";
    private static final String ROBOT_SERIAL_MILLENNIUM_SWERVE      = "32d62e829b0befa2";

    private static String robotName = "Unknown";

    private static boolean isSwerveChassis;
    private static boolean isMecanumChassis;
    private static double xPodOffset;
    private static double yPodOffset;
    private static ForesightConfig foresightConfig;

    @SuppressWarnings("unused")
    public static String getRobotName() {
        init();

        return robotName;
    }

    private static void init() {
        if (initialized) return;

        String robotSerialNumber = Device.getSerialNumberOrUnknown();

        switch (robotSerialNumber) {
            case ROBOT_SERIAL_COACH_ANTHONY:
                initCoachAnthonyRobot();
                break;
            case ROBOT_SERIAL_MILLENNIUM_STARTER_BOT:
                initMillenniumStarterRobot();
                break;
            case ROBOT_SERIAL_MILLENNIUM_SWERVE:
                initMillenniumSwerveRobot();
                break;
            default:
                // Unknown robot
                break;
        }

        initialized = true;
    }

    private static void initMillenniumSwerveRobot() {
        robotName = "Millennium Swerve";

        isSwerveChassis = true;
        isMecanumChassis = false;
        xPodOffset = 96.0;
        yPodOffset = -194.5;
        foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15054265544715764);
                Controller secondaryTranslationalForward = Controller.proportional(0.0556214886138864);
                Controller primaryTranslationalLateral = Controller.proportional(0.1972822375468673);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07289051529509401);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016223893876644743));
                c.brake.set(Controller.proportionalFeedforward(0.013790309795148031));

                c.headingFeedback.set(Controller.proportional(2.7109960647229423));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03923918500290919, 0.005238250122071822));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06691777859993613, 0.05651738468443017));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010499120508739041, 0.001184877917208147));

                c.maxAchievableForwardVelocity.set(63.005591468932366);
                c.maxAchievableStrafeVelocity.set(53.046979792969296);
                c.naturalForwardDeceleration.set(35.00835406004796);
                c.naturalStrafeDeceleration.set(49.609008938604696);
            }
        );
    }

    private static void initMillenniumStarterRobot() {
        robotName = "Millennium Starter";

        isMecanumChassis = true;
        isSwerveChassis = false;
        xPodOffset = -84.0;
        yPodOffset = -168.0;
        foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15054265544715764);
                Controller secondaryTranslationalForward = Controller.proportional(0.0556214886138864);
                Controller primaryTranslationalLateral = Controller.proportional(0.1972822375468673);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07289051529509401);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016223893876644743));
                c.brake.set(Controller.proportionalFeedforward(0.013790309795148031));

                c.headingFeedback.set(Controller.proportional(2.7109960647229423));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03923918500290919, 0.005238250122071822));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06691777859993613, 0.05651738468443017));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010499120508739041, 0.001184877917208147));

                c.maxAchievableForwardVelocity.set(63.005591468932366);
                c.maxAchievableStrafeVelocity.set(53.046979792969296);
                c.naturalForwardDeceleration.set(35.00835406004796);
                c.naturalStrafeDeceleration.set(49.609008938604696);
            }
        );
    }

    private static void initCoachAnthonyRobot() {
        robotName = "Coach Anthony";

        isMecanumChassis = true;
        isSwerveChassis = false;
        xPodOffset = -84.0;
        yPodOffset = -168.0;
        foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15054265544715764);
                Controller secondaryTranslationalForward = Controller.proportional(0.0556214886138864);
                Controller primaryTranslationalLateral = Controller.proportional(0.1972822375468673);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07289051529509401);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016223893876644743));
                c.brake.set(Controller.proportionalFeedforward(0.013790309795148031));

                c.headingFeedback.set(Controller.proportional(2.7109960647229423));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03923918500290919, 0.005238250122071822));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06691777859993613, 0.05651738468443017));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010499120508739041, 0.001184877917208147));

                c.maxAchievableForwardVelocity.set(63.005591468932366);
                c.maxAchievableStrafeVelocity.set(53.046979792969296);
                c.naturalForwardDeceleration.set(35.00835406004796);
                c.naturalStrafeDeceleration.set(49.609008938604696);
            }
        );
    }

    public static boolean isSwerveChassis() {
        init();

        return isSwerveChassis;
    }

    public static boolean isMecanumChassis() {
        init();

        return isMecanumChassis;
    }

    public static  ForesightConfig getForesightConfig() {
        init();

        return foresightConfig;
    }

    public static double getXPodOffset() {
        init();

        return xPodOffset;
    }

    public static double getYPodOffset() {
        init();

        return yPodOffset;
    }
}
