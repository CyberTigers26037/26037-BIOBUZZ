package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.servos.AxonServo;

@SuppressWarnings("unused")
@TeleOp
public class HiveFinderTestOpMode1 extends OpMode {
    private static final Pose startPose = PoseFactory.degrees() .of(132, 9, 90);

    private static final double FIELD_CENTER = 72;
    private static final double HIVE_OFFSET_X = 12.75;
    private static final double HIVE_OFFSET_Y = 19.75;
    private static final Pose hiveBlueAudience = PoseFactory.degrees() .of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);


    private Follower follower;
    private AxonServo servo;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        servo = new AxonServo(hardwareMap.get(Servo.class, "pointerServo"), null, true);

    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()){
            servo.setTargetAngle(0);
        }
        else if (gamepad1.dpadUpWasPressed()){
            servo.setTargetAngle(90);
        }
        else if (gamepad1.dpadLeftWasPressed()){
            servo.setTargetAngle(180);
        }
        else if (gamepad1.dpadDownWasPressed()){
            servo.setTargetAngle(270);
        }
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();

        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

        double angleToTarget = calculateAngleToTarget(robotPose, hiveBlueAudience);
        telemetry.addData("Target Heading", angleToTarget);
        servo.setTargetAngle(angleToTarget);
    }
    private double calculateAngleToTarget(Pose robotPose, Pose targetPose) {
        double deltaX = targetPose.x() - targetPose.x();
        double deltaY = targetPose.y() - targetPose.y();
        double absoluteTargetHeading = Math.atan2(deltaY, deltaX);
        double relativeTargetHeading = absoluteTargetHeading - robotPose.heading();
        while (relativeTargetHeading > Math.PI) relativeTargetHeading -= 2 * Math.PI;
        while (relativeTargetHeading < -Math.PI) relativeTargetHeading += 2 * Math.PI;
        double turretAngleRad = (Math.PI / 2.0) + relativeTargetHeading;
        double TWO_PI = 2.0 * Math.PI;
        turretAngleRad = (turretAngleRad % TWO_PI + TWO_PI) % TWO_PI;
        return Math.toDegrees(turretAngleRad);

    }
}