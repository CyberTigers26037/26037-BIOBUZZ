package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassembly.Intake;

@SuppressWarnings("unused")
@TeleOp(name = "IntakeTestOpMode", group = "Test")

public class IntakeTestOpMode extends OpMode {
    private Intake intake;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.aWasPressed()){
            if(!intake.isRunning()){
                intake.start();
            }
            else{
                intake.stop();
            }

        }
        else if(gamepad1.bWasPressed()) {
            if(!intake.isRunning()){
                intake.startReverse();
            }
            else{
                intake.stop();
            }
        }


        telemetry.addData("Intake: ", intake.isRunning());
        telemetry.addData("Reverse Intake: ", intake.isRunningInReverse());
    }
}