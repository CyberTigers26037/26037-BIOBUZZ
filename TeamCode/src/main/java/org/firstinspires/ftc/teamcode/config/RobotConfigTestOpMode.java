package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Device;

@TeleOp
@SuppressWarnings("unused")
public class RobotConfigTestOpMode extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Serial: ", Device.getSerialNumberOrUnknown());
        telemetry.addData("Robot: ", RobotConfig.getRobotName());
    }

    @Override
    public void loop() {

    }
}
