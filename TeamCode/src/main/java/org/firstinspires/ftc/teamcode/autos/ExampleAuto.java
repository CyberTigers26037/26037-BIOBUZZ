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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class ExampleAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(132, 9, 90);
    private final Pose scorePose = poseFactory.of(132, 48, 90);
    private final Pose medianPose = poseFactory.of(132, 96, 90);
    private final Pose parkPose = poseFactory.of(96, 120, 180);

    // Path methods
    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    private Path park(){
        return curve(scorePose, medianPose, parkPose).linear(scorePose, parkPose);
    }

    private final Pose startPose2 = poseFactory.of(96, 120, 180);
//    private final Pose rotatePose2 = poseFactory.of(96, 121, 0);
      private final Pose medianPose2 = poseFactory.of( 132, 96, 90);
      private final Pose scorePose2 = poseFactory.of(132, 48, 90);
      private final Pose parkPose2 = poseFactory.of(132, 9, 90);

      private Path startToScore2() {
          return curve(startPose2, medianPose2, scorePose2).linear(startPose2, scorePose2);
      }
//      private Path rotatePose2() {
//          return line(startPose2, rotatePose2).linear(startPose2, rotatePose2);
//      }

      private Path park2() {
          return line(scorePose2, parkPose2).linear(scorePose2, parkPose2);
      }
//

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park()),
                follow(follower, startToScore2()),
//                follow(follower, rotatePose2()),
                follow(follower, park2())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
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
}
