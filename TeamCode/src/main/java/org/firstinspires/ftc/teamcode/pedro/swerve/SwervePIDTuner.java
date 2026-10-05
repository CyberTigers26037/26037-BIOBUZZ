package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.controllers.Controller;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.util.Timer;

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

    private TelemetryManager telemetryM;
    public static double targetAngle = 0;
    private CoaxialPod pod;
    private CoaxialPodConfig config;
    private Timer inputTimer;

    public static double P;
    public static double I;
    public static double D;
    public static double F;

    private TUNING_PARAMETER tuningParameter = TUNING_PARAMETER.P;

    @Override
    public void init() {
        inputTimer = new Timer(0.25, true);
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
        if ((gamepad1.left_stick_y < -0.5) && inputTimer.isExpired()) {
            tuningAdjustment = 1;
            inputTimer.start();
        }
        else if ((gamepad1.left_stick_y > 0.5) && inputTimer.isExpired()) {
            tuningAdjustment = -1;
            inputTimer.start();
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

        pod.move(targetAngle, 0, false);

        telemetry.addData("Currently Tuning", tuningParameter);
        telemetry.addLine("Left Stick Y to Adjust");
        telemetry.addLine();

        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetryM.addData("Target Rotation", Math.toDegrees(targetAngle));
        telemetryM.addData("Actual Rotation", Math.toDegrees(pod.getOffsetAngleRad()));
    }

    private enum TuningMode {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    }
}

