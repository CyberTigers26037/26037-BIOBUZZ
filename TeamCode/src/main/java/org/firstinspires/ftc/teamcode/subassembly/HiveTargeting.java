package org.firstinspires.ftc.teamcode.subassembly;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HiveTargeting {
    private static final double FIELD_CENTER = 72;
    private static final double HIVE_OFFSET_X = 12.75;
    private static final double HIVE_OFFSET_Y = 19.75;
    private static final Pose hiveBlueAudience = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private static final Pose hiveBlueScoring = PoseFactory.degrees().of(FIELD_CENTER + HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveRedScoring = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER + HIVE_OFFSET_Y, 0);
    private static final Pose hiveRedAudience = PoseFactory.degrees().of(FIELD_CENTER - HIVE_OFFSET_X, FIELD_CENTER - HIVE_OFFSET_Y, 0);
    private Follower follower;

    private NumberPlateSensor numberPlateSensor;

    public double lastAngleToTarget;

    public HiveTargeting(HardwareMap hardwareMap,Follower follower) {
        numberPlateSensor = new NumberPlateSensor(hardwareMap);
        this.follower = follower;
    }

    public double getAngleToTarget() {
        Pose robotPose = follower.pose();

        double hiveBlueAudienceDistance = calculateDistanceToTarget(robotPose, hiveBlueAudience);
        double hiveBlueScoringDistance = calculateDistanceToTarget(robotPose, hiveBlueScoring);
        double hiveRedAudienceDistance = calculateDistanceToTarget(robotPose, hiveRedAudience);
        double hiveRedScoringDistance = calculateDistanceToTarget(robotPose, hiveRedScoring);

        if (numberPlateSensor.isNumberPlateBlue()) {
            Pose target;
            if (hiveBlueScoringDistance < hiveBlueAudienceDistance) {
                target = hiveBlueScoring;
            } else {
                target = hiveBlueAudience;
            }
            lastAngleToTarget = calculateAngleToTarget(robotPose, target);
        } else {
            Pose targetB;
            if (hiveRedScoringDistance < hiveRedAudienceDistance) {
                targetB = hiveRedScoring;
            } else {
                targetB = hiveRedAudience;
            }
            lastAngleToTarget = calculateAngleToTarget(robotPose, targetB);
        }
        return lastAngleToTarget;
    }
        public void outputTelemetry(Telemetry telemetry){
            telemetry.addData("Robot X", follower.pose().x());
            telemetry.addData("Robot Y", follower.pose().y());
            telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Target Heading Blue", lastAngleToTarget);

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