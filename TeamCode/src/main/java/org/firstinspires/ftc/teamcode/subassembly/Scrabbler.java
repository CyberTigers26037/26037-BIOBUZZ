package org.firstinspires.ftc.teamcode.subassembly;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.servos.AxonServo;

public class Scrabbler {
    private static final int UP_ANGLE = 0;
    private static final int DOWN_ANGLE = 90;
    private AxonServo servo;
    public Scrabbler(HardwareMap hardwareMap) {
        Servo servo = hardwareMap.get(Servo.class, "scrabbler");
        this.servo = new AxonServo(servo, null, false);
    }
    public void raise() {
        servo.setTargetAngle(UP_ANGLE);
    }
    public void lower() {
        servo.setTargetAngle(DOWN_ANGLE);
    }
}

