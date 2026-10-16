package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subassembly.LauncherTurret;

@SuppressWarnings("unused")
@TeleOp
public class LauncherturretTestOpMode extends OpMode {
    private LauncherTurret launcherTurret1;
    private LauncherTurret launcherTurret2;
    private double turretAngle = 0;
    private float lastrightangle = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();


    @Override
    public void init() {

        launcherTurret1 = new LauncherTurret(hardwareMap, "1");
        launcherTurret2 = new LauncherTurret(hardwareMap, "2");
    }

    @Override
    public void loop() {
        telemetry.addData("leftjoy", gamepad1.left_stick_x);
        telemetry.addData("turretangle", turretAngle);
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (Math.abs( gamepad1.left_stick_x) > 0.01) {
            turretAngle += gamepad1.left_stick_x * 90 * dt;
            turretAngle = Range.clip(turretAngle, 0, 359);
        }

        if (gamepad1.dpadRightWasPressed()) {
            turretAngle = 0;
        }
        if (gamepad1.dpadUpWasPressed()) {
            turretAngle = 90;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            turretAngle = 180;
        }
        if (gamepad1.dpadDownWasPressed()) {
            turretAngle = 270;
        }
        if (gamepad1.rightTriggerWasPressed()) {
            launcherTurret1.toggleLauncher();
            launcherTurret2.toggleLauncher();
        }
        if (gamepad1.aWasPressed()){
            int currpm = launcherTurret1.getLauncherRpm();
            launcherTurret1.setLauncherRpm(currpm - 100);
            launcherTurret2.setLauncherRpm(currpm - 100);
        }
        if (gamepad1.yWasPressed()){
            int currpm = launcherTurret1.getLauncherRpm();
            launcherTurret1.setLauncherRpm(currpm + 100);
            launcherTurret2.setLauncherRpm(currpm + 100);
        }
        launcherTurret1.setTurretAngle(turretAngle);
        launcherTurret2.setTurretAngle(turretAngle);
        launcherTurret1.loop();
        launcherTurret2.loop();

        launcherTurret1.outputTelemetry(telemetry);
        launcherTurret2.outputTelemetry(telemetry);
    }
}
