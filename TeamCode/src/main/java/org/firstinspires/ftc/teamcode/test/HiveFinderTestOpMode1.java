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
    private static final Pose hiveBlueAudience = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private static final Pose hiveBlueScoring = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveRedScoring = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveRedAudience = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);

    private boolean isBlueAlliance;
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

        if (gamepad1.bWasPressed()) {
            isBlueAlliance = false;
        }
        else if (gamepad1.xWasPressed()){
            isBlueAlliance = true;
        }

        telemetry.addData("Blue Alliance", isBlueAlliance);


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

        double hiveBlueAudienceDistance = calculateDistanceToTarget(robotPose, hiveBlueAudience);
        double hiveBlueScoringDistance = calculateDistanceToTarget(robotPose, hiveBlueScoring);
        double hiveRedAudienceDistance = calculateDistanceToTarget(robotPose, hiveRedAudience);
        double hiveRedScoringDistance = calculateDistanceToTarget(robotPose, hiveRedScoring);

        if (isBlueAlliance) {
            Pose target;
            if (hiveBlueScoringDistance < hiveBlueAudienceDistance) {
                target = hiveBlueScoring;
            } else {
                target = hiveBlueAudience;
            }
            double angleToTarget = calculateAngleToTarget(robotPose, target);
            telemetry.addData("Target Heading Blue", angleToTarget);
            servo.setTargetAngle(angleToTarget);
        }
        else {
            Pose targetB;
            if (hiveRedScoringDistance < hiveRedAudienceDistance) {
                targetB = hiveRedScoring;
            } else {
                targetB = hiveRedAudience;
            }
            double angleToTargetB = calculateAngleToTarget(robotPose, targetB);
            telemetry.addData("Target Heading Red", angleToTargetB);
            servo.setTargetAngle(angleToTargetB);
        }
        }





    private double calculateAngleToTarget(Pose robotPose, Pose targetPose) {
        double deltaX = targetPose.x() - robotPose.x();
        double deltaY = targetPose.y() - robotPose.y();
        double absoluteTargetHeading = Math.atan2(deltaY, deltaX);

        double relativeTargetHeading = absoluteTargetHeading - robotPose.heading();

        while (relativeTargetHeading > Math.PI) relativeTargetHeading -= 2 * Math.PI;
        while (relativeTargetHeading < -Math.PI) relativeTargetHeading += 2 * Math.PI;

        double turretAngleRad = (Math.PI / 2.0) + relativeTargetHeading;

        double TWO_PI = 2.0 * Math.PI;
        turretAngleRad = (turretAngleRad % TWO_PI + TWO_PI) % TWO_PI;

        return Math.toDegrees(turretAngleRad);
    }

    private double calculateDistanceToTarget(Pose robotPose, Pose targetPose) {
        return Math.hypot(targetPose.x() - robotPose.x(), targetPose.y() - robotPose.y());
    }
}
