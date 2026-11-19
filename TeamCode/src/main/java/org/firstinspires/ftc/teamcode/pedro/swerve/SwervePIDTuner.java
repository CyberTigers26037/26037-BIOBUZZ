package org.firstinspires.ftc.teamcode.pedro.swerve;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.IOException;
import java.util.List;

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
    private DataLogger logger;
    private List<LynxModule> allHubs;

    public static double P = 0.35;
    public static double I = 0.0;
    public static double D = 0.0325;
    public static double F = 0.06;
    public static double A = 0.4;
    public static double ANGLE_DELTA = 45;

    @Override
    public void init() {
        try {
            logger = new DataLogger("swerve_pid_log");
        } catch (IOException e) {
            logger = null;
        }

        setupPodSelection();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void setupPodSelection() {
        switch (podSelection) {
            case LEFT_FRONT:
                config = Constants.leftFront;
                pod = new CoaxialPod(hardwareMap, Constants.leftFront, logger);
                break;
            case RIGHT_FRONT:
                config = Constants.rightFront;
                pod = new CoaxialPod(hardwareMap, Constants.rightFront, logger);
                break;
            case LEFT_BACK:
                config = Constants.leftBack;
                pod = new CoaxialPod(hardwareMap, Constants.leftBack, logger);
                break;
            case RIGHT_BACK:
                config = Constants.rightBack;
                pod = new CoaxialPod(hardwareMap, Constants.rightBack, logger);
                break;
        }

        pidController = new SwervePIDFController(P, I, D, F, A, logger);
        config.turnController.set(pidController);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

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

        pod.move(targetAngle, 0, false);

        telemetry.addData("Currently Tuning", podSelection);
        telemetry.addLine();

        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetryM.addData("error", pidController.getPreviousError());
        telemetryM.update();
    }

    @Override
    public void stop() {
        if (logger != null) {
            logger.close();
        }
    }

    private enum PodSelection {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    }
}
