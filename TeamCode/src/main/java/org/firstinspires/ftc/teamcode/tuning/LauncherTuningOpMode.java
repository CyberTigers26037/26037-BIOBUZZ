package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@SuppressWarnings("unused")
@TeleOp(group = "Tuning")
public class LauncherTuningOpMode extends OpMode {
    private enum MotorSelection {
        FLYWHEEL1,
        FLYWHEEL2
    }

    private static final int MIN_RPM =  100;
    private static final int MAX_RPM = 6000;
    private static final double MOTOR_TICKS_PER_REVOLUTION = 28;
    private static final double SECONDS_PER_MINUTE = 60;

    public static double P = 10;
    public static double I = 3;
    public static double D = 0;
    public static double F = 0;

    private MotorSelection currentMotorSelection;
    private TelemetryManager telemetryM;
    private int motorRpm = 3000;

    private DcMotorEx motor;

    @Override
    public void init() {
        initMotor(MotorSelection.FLYWHEEL1);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    private void initMotor(MotorSelection motorSelection) {
        motor = hardwareMap.get(DcMotorEx.class, motorSelection == MotorSelection.FLYWHEEL1 ? "flywheelMotor1" : "flywheelMotor2");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidf = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        P = pidf.p;
        I = pidf.i;
        D = pidf.d;
        F = pidf.f;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        this.currentMotorSelection = motorSelection;
    }

    @Override
    public void loop() {
        MotorSelection newMotorSelection = currentMotorSelection;
        if (gamepad1.xWasPressed()) newMotorSelection = MotorSelection.FLYWHEEL1;
        if (gamepad1.yWasPressed()) newMotorSelection = MotorSelection.FLYWHEEL2;
        if (newMotorSelection != currentMotorSelection) {
            initMotor(newMotorSelection);
        }

        if (gamepad1.dpadDownWasPressed()) {
            motorRpm -= 100;
        }
        if (gamepad1.dpadUpWasPressed()) {
            motorRpm += 100;
        }

        telemetry.addLine("DPAD Up/Down to adjust Target RPM");
        telemetry.addLine("Right Trigger to run motor");
        telemetry.addLine("X to tune Motor 1, Y to tune Motor 2");
        telemetry.addLine();

        PIDFCoefficients pidf = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        if (updatePIDF(pidf)) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }

        if (gamepad1.right_trigger > 0.1) {
            motor.setVelocity(rpmToTps(motorRpm));
        }
        else {
            motor.setVelocity(0);
        }

        telemetry.addData("Target RPM", motorRpm);
        telemetry.addData("Actual RPM", tpsToRpm(motor.getVelocity()));
        telemetry.addLine();

        telemetry.addData("P", pidf.p);
        telemetry.addData("I", pidf.i);
        telemetry.addData("D", pidf.d);
        telemetry.addData("F", pidf.f);

        telemetryM.addData("Target RPM", motorRpm);
        telemetryM.addData("Actual RPM", tpsToRpm(motor.getVelocity()));
        telemetryM.update();
    }

    private boolean updatePIDF(PIDFCoefficients pidf) {
        if ((pidf.p != P) || (pidf.i != I) || (pidf.d != D) || (pidf.f != F)) {
            pidf.p = P;
            pidf.i = I;
            pidf.d = D;
            pidf.f = F;
            return true;
        }

        return  false;
    }

    private double rpmToTps(int rpm) {
        // Converts revolutions per minute (RPM) to ticks per second (TPS)
        return rpm * MOTOR_TICKS_PER_REVOLUTION / SECONDS_PER_MINUTE;
    }

    private int tpsToRpm(double tps) {
        // Converts ticks per second (TPS) to revolutions per minute (RPM)
        return (int)(tps / MOTOR_TICKS_PER_REVOLUTION * SECONDS_PER_MINUTE);
    }
}
