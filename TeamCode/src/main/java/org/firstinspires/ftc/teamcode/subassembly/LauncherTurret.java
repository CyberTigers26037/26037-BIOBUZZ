package org.firstinspires.ftc.teamcode.subassembly;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.servos.RTPAxon;

public class LauncherTurret {
    private DigitalChannel limitSwitch;
    private boolean initialized;
    private RTPAxon turretServo;

    public LauncherTurret(HardwareMap hardwareMap) {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        CRServo servo = hardwareMap.get(CRServo.class, "turretServo");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        turretServo = new RTPAxon(servo, encoder, 0, false);
        turretServo.setMaxPower(1.0);
        turretServo.setPidCoeffs(0.007, 0.00041, 0.0002);
    }

    public void loop() {
        telemetry.addData("limitSwitch", limitSwitch.getState());
        if (!initialized) {
            turretServo.setPower(-0.1);

            if (limitSwitch.getState()) {
                turretServo.setPower(0);
                turretServo.resetHomeAngle();
                initialized = true;
            }
            return;
        }
        turretServo.update();
    }
    public void setTurretAngle(double angle) {
            turretServo.setTargetRotation(angle*2.75);
    }

    public void outputTelemetry(Telemetry telemetry) {
        telemetry.addLine(turretServo.log());
    }
}
