package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestLight extends OpMode {
    private Servo rgbLight;
    private static final double BLUE = 0.611;
    @Override
    public void init() {
        rgbLight = hardwareMap.tryGet(Servo.class, "light");
    }

    @Override
    public void loop() {
        rgbLight.setPosition(BLUE);

    }
}
