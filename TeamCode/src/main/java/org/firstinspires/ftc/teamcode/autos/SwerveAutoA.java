package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subassembly.Intake;
import org.firstinspires.ftc.teamcode.subassembly.NumberPlateSensor;

@Autonomous
public class SwerveAutoA extends OpMode {
    private Follower follower;
    private Intake intake;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    protected boolean isBlueAlliance;


    // Poses
    private Pose startPose;
    private Pose armDropPose;
    private Pose medianPose;
    private Pose parkPose;

    // Path methods
    private Path startToArmDrop() {
        return line(startPose, armDropPose).linear(startPose, armDropPose);
    }
    private Path armDropToPark() {
        return curve(armDropPose, medianPose, parkPose).linear(armDropPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToArmDrop()),
                follow(follower, armDropToPark() )
                //intake.getStartIntakeCommand(),
                //intake.getStopIntakeCommand()
        );
    }

    private void setUpPoses() {
        if(isBlueAlliance) {
            startPose = poseFactory.of(82, 131, 270);
            armDropPose = poseFactory.of(82, 127, 270);
            medianPose = poseFactory.of(132, 96, 270);
            parkPose = poseFactory.of(129, 48, 270);
        }
        else {
            startPose = poseFactory.of(62, 13, 90);
            armDropPose = poseFactory.of(62, 17, 90);
            medianPose = poseFactory.of(12, 48, 90);
            parkPose = poseFactory.of(14, 96, 90);
        }
    }

    @Override
    public void init() {
        NumberPlateSensor numberPlateSensor = new NumberPlateSensor(hardwareMap);
        isBlueAlliance = numberPlateSensor.isNumberPlateBlue();

        setUpPoses();

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}
