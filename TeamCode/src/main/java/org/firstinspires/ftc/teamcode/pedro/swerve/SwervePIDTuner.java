package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.controllers.Controller;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * This is the SwervePIDTuner
 * You should use this to tune your swerve pod PID values
 * @author Havish Sripada
 */
@SuppressWarnings("unused")
@TeleOp
public class SwervePIDTuner extends OpMode {
    private enum TUNING_PARAMETER { P, I, D }
    private static final double TUNING_ADJUSTMENT_P = 0.1;
    private static final double TUNING_ADJUSTMENT_I = 0.01;
    private static final double TUNING_ADJUSTMENT_D = 0.005;
    private static final TuningMode mode = TuningMode.LEFT_FRONT;
    private static final int INPUT_TIMEOUT_MILLIS = 250;

    private TelemetryManager telemetryM;
    public static double targetAngle = 0;
    private CoaxialPod pod;
    private CoaxialPodConfig config;
    private long lastInputMillis;

    public static double P  = 0.3;
    public static double I = 0.0;
    public static double D = 0.005;
    public static double F;

    private TUNING_PARAMETER tuningParameter = TUNING_PARAMETER.P;

    @Override
    public void init() {
        switch (mode) {
            case LEFT_FRONT:
                config = Constants.leftFront;
                pod = new CoaxialPod(hardwareMap, Constants.leftFront);
                break;
            case RIGHT_FRONT:
                config = Constants.rightFront;
                pod = new CoaxialPod(hardwareMap, Constants.rightFront);
                break;
            case LEFT_BACK:
                config = Constants.leftBack;
                pod = new CoaxialPod(hardwareMap, Constants.leftBack);
                break;
            case RIGHT_BACK:
                config = Constants.rightBack;
                pod = new CoaxialPod(hardwareMap, Constants.rightBack);
                break;
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private double convertTargetAngle(double targetAngleRad) {
        boolean encoderReversed = config.encoderReversed.get();

        // Convert hardware angle to radians and normalize
        double actualRad = pod.getAngleAfterOffsetRad();
        actualRad = Angle.normalize(actualRad);

        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double desiredRad = encoderReversed ? targetAngleRad : (2 * Math.PI - targetAngleRad);
        desiredRad += Math.PI / 2.0;
        desiredRad = Angle.normalize(desiredRad);

        // Shortest-path error in radians (signed)
        double mag = Angle.smallestDifference(actualRad, desiredRad);
        double dir = Angle.turnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // PID uses radians (tune PIDF for radian error)
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90°
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            // add 180 degrees (pi radians)
            desiredRad = Angle.normalize(desiredRad + Math.PI);

            // recompute signed error
            mag = Angle.smallestDifference(actualRad, desiredRad);
            dir = Angle.turnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        return actualRad + errorRad;
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) tuningParameter = TUNING_PARAMETER.P;
        if (gamepad1.yWasPressed()) tuningParameter = TUNING_PARAMETER.I;
        if (gamepad1.bWasPressed()) tuningParameter = TUNING_PARAMETER.D;

        telemetry.addLine("Dpad to set target angle: U 0, R 90, D 180, L 270");
        telemetry.addLine("X to tune P, Y to tune I, B to tune D");
        telemetry.addLine("Right Stick Y to drive motor");
        telemetry.addLine();

        double tuningAdjustment;
        boolean inputTimerExpired = System.currentTimeMillis() - lastInputMillis > INPUT_TIMEOUT_MILLIS;
        if ((gamepad1.left_stick_y < -0.5) && inputTimerExpired) {
            tuningAdjustment = 1;
            lastInputMillis = System.currentTimeMillis();
        }
        else if ((gamepad1.left_stick_y > 0.5) && inputTimerExpired) {
            tuningAdjustment = -1;
            lastInputMillis = System.currentTimeMillis();
        }
        else {
            tuningAdjustment = 0;
        }

        if (tuningAdjustment != 0) {
            switch (tuningParameter) {
                case P:
                    P += (tuningAdjustment * TUNING_ADJUSTMENT_P);
                    break;
                case I:
                    I += (tuningAdjustment * TUNING_ADJUSTMENT_I);
                    break;
                case D:
                    D += (tuningAdjustment * TUNING_ADJUSTMENT_D);
                    break;
            }
        }

        config.turnController.set(Controller.pid(P, I, D).plus(Controller.proportionalFeedforward(F)));

        if (gamepad1.dpadUpWasPressed()) {
            targetAngle = Math.toRadians(0);
        }
        if (gamepad1.dpadRightWasPressed()) {
            targetAngle = Math.toRadians(90);
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetAngle = Math.toRadians(180);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetAngle = Math.toRadians(270);
        }

        double podCorrectedTargetAngle = convertTargetAngle(targetAngle);

        pod.move(targetAngle, 0, false);

        telemetry.addData("Currently Tuning", tuningParameter);
        telemetry.addLine("Left Stick Y to Adjust");
        telemetry.addLine();

        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetryM.addData("Target Rotation", Math.toDegrees(targetAngle));
        telemetryM.addData("Target Pod Rotation", Math.toDegrees(podCorrectedTargetAngle));
        telemetryM.addData("Actual Rotation", Math.toDegrees(pod.getOffsetAngleRad()));
        telemetryM.update();
    }

    private enum TuningMode {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    }
}

