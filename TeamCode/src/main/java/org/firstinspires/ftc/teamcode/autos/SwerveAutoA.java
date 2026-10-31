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

@Autonomous
public class SwerveAutoA extends OpMode {
    private Follower follower;
    private Intake intake;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(82, 131, 270);
    private final Pose armDropPose = poseFactory.of(82, 127, 270);

    private final Pose medianPose = poseFactory.of(132, 96, 270);
    private final Pose parkPose = poseFactory.of(129, 48, 270);

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

    @Override
    public void init() {
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
