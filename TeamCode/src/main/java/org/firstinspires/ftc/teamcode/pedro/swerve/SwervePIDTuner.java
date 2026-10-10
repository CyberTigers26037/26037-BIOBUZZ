package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
@Configurable
@SuppressWarnings("unused")
@TeleOp
public class SwervePIDTuner extends OpMode {
    private static final int INPUT_TIMEOUT_MILLIS = 250;

    private TelemetryManager telemetryM;
    public static double targetAngle = 0;
    private CoaxialPod pod;
    private CoaxialPodConfig config;
    private long lastInputMillis;
    private SwervePIDFController pidController;
    private PodSelection podSelection = PodSelection.RIGHT_FRONT;

    public static double P = 0.35;
    public static double I = 0.0;
    public static double D = 0.020;
    public static double F = 0.025;
    public static double A = 0.3;
    public static double ANGLE_DELTA = 90;

    @Override
    public void init() {
        setupPodSelection();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void setupPodSelection() {
        switch (podSelection) {
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

        pidController = new SwervePIDFController(P, I, D, F, A);
        config.turnController.set(pidController);
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

        telemetryM.addData("actualRad", Math.toDegrees(actualRad));
        telemetryM.addData("errorRad", Math.toDegrees(errorRad));

        return actualRad + errorRad;
    }

    @Override
    public void loop() {
        PodSelection newPodSelection = podSelection;
        if (gamepad1.xWasPressed()) podSelection = PodSelection.LEFT_FRONT;
        if (gamepad1.yWasPressed()) podSelection = PodSelection.RIGHT_FRONT;
        if (gamepad1.aWasPressed()) podSelection = PodSelection.LEFT_BACK;
        if (gamepad1.bWasPressed()) podSelection = PodSelection.RIGHT_BACK;
        if (newPodSelection != podSelection) {
            setupPodSelection();
        }

        telemetry.addLine("Dpad to set target angle: U 0, R 90, D 180, L 270");
        telemetry.addLine("X to tune LF, Y to tune RF, A to tune LB, B to tune RB");
        telemetry.addLine();

        pidController.setPIDF(P, I, D, F, A);

        if (gamepad1.dpadUpWasPressed()) {
            targetAngle = Math.toRadians(0);
            pidController.reset();
        }
        if (gamepad1.dpadRightWasPressed()) {
            targetAngle = Math.toRadians(ANGLE_DELTA);
            pidController.reset();
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetAngle = Math.toRadians(ANGLE_DELTA*2);
            pidController.reset();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetAngle = Math.toRadians(ANGLE_DELTA*3);
            pidController.reset();
        }

        double podCorrectedTargetAngle = convertTargetAngle(targetAngle);

        pod.move(targetAngle, 0, false);

        telemetry.addData("Currently Tuning", podSelection);
        telemetry.addLine();

        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetryM.addData("Target Rotation", Math.toDegrees(targetAngle));
        telemetryM.addData("Target Pod Rotation", Math.toDegrees(podCorrectedTargetAngle));
        telemetryM.addData("Actual Rotation", Math.toDegrees(pod.getOffsetAngleRad()));
        telemetryM.update();
    }

    private enum PodSelection {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    }
}

