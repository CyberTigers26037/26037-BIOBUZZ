package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subassembly.NumberPlateSensor;

@Autonomous
public class SwerveAutoB extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    protected boolean isBlueAlliance;

    // Poses
    private Pose startPose;
    private Pose armDropPose;

    private  Pose parkPose;

    // Path methods
    private Path startToArmDrop() {
        return line(startPose, armDropPose).linear(startPose, armDropPose);
    }
    private Path armDropToPark() {
        return line(armDropPose, parkPose).linear(armDropPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToArmDrop()),
                follow(follower, armDropToPark())
        );
    }


    private void setUpPoses() {
        if(isBlueAlliance) {
            startPose = poseFactory.of(120, 9, 90);
            armDropPose = poseFactory.of(120, 13, 90);
            parkPose = poseFactory.of(130, 22, 90);
        }
        else {
            startPose = poseFactory.of(24, 135, 270);
            armDropPose = poseFactory.of(24, 131, 270);
            parkPose = poseFactory.of(14, 122, 270);
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

    // in your autonomous


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}
