package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassembly.HiveAprilTagLimelight;

@TeleOp(name="LimelightTestOpMode", group = "Test")
public class LimelightTestOpMode extends OpMode {
    private HiveAprilTagLimelight limelight;

    @Override
    public void init() {
        limelight = new HiveAprilTagLimelight();
        limelight.init(hardwareMap);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.loop(telemetry);
    }
}