package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassembly.LauncherTurret;

@SuppressWarnings("unused")
@TeleOp
public class LauncherturretTestOpMode extends OpMode {
    private LauncherTurret launcherTurret1;
    private LauncherTurret launcherTurret2;
    private float lastleftangle = 0;
    private float lastrightangle = 0;


    @Override
    public void init() {

        launcherTurret1 = new LauncherTurret(hardwareMap, "1");
        launcherTurret2 = new LauncherTurret(hardwareMap, "2");
    }

    @Override
    public void loop() {
        telemetry.addData("leftjoy", gamepad1.left_stick_x);
        telemetry.addData("leftlast", lastleftangle);

        if (gamepad1.left_stick_x != 0.0) {
            lastleftangle = 30f;
            if (lastleftangle >= 360) lastleftangle = 0;
            if (lastleftangle < 0) lastleftangle = 359;
            launcherTurret1.setTurretAngle(lastleftangle);
        }

        if (gamepad1.dpadRightWasPressed()) {
            launcherTurret1.setTurretAngle(0);
            lastleftangle = 0;
            launcherTurret2.setTurretAngle(0);
        }
        if (gamepad1.dpadUpWasPressed()) {
            launcherTurret1.setTurretAngle(90);
            lastleftangle = 90;
            launcherTurret2.setTurretAngle(90);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcherTurret1.setTurretAngle(180);
            lastleftangle = 180;
            launcherTurret2.setTurretAngle(180);
        }
        if (gamepad1.dpadDownWasPressed()) {
            launcherTurret1.setTurretAngle(270);
            lastleftangle = 270;
            launcherTurret2.setTurretAngle(270);
        }
        if (gamepad1.rightTriggerWasPressed()) {
            launcherTurret1.toggleLauncher();
            launcherTurret2.toggleLauncher();
        }
        launcherTurret1.loop();
       // launcherTurret2.loop();

        launcherTurret1.outputTelemetry(telemetry);
        launcherTurret2.outputTelemetry(telemetry);
    }
}
