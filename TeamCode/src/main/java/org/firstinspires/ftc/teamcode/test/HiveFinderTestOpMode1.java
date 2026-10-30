package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.servos.AxonServo;
import org.firstinspires.ftc.teamcode.subassembly.HiveTargeting;

@SuppressWarnings("unused")
@TeleOp(group = "Test")
public class HiveFinderTestOpMode1 extends OpMode {
    private static final Pose startPose = PoseFactory.degrees() .of(120, 9, 90);

    private HiveTargeting hiveTargeting;
    private Follower follower;
    private AxonServo servo;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        //servo = new AxonServo(hardwareMap.get(Servo.class, "pointerServo"), null, true);
        hiveTargeting = new HiveTargeting(hardwareMap, follower);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();
        double angleToTarget = hiveTargeting.getAngleToTarget();
        //servo.setTargetAngle(angleToTarget);
        hiveTargeting.outputTelemetry(telemetry);
        }


}
