package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subassembly.TurretSystem;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends OpMode {

    private Follower follower;
    private TurretSystem turretSystem;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        turretSystem = new TurretSystem(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
        telemetry.addData("leftjoy", gamepad1.left_stick_x);

        if (gamepad1.rightTriggerWasPressed()) {
            turretSystem.toggleLauncher();
        }
        if (gamepad1.aWasPressed()){
            turretSystem.adjustLauncherRpm(- 100);
        }
        if (gamepad1.yWasPressed()){
            turretSystem.adjustLauncherRpm(+ 100);
        }
        turretSystem.loop(gamepad1.left_stick_x);
        turretSystem.outputTelemetry(telemetry);
    }

}
