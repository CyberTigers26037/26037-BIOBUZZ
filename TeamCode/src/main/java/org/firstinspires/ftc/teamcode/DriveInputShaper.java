package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DriveInputShaper{
    private static final double MAX_RATE = 4.0;
    private double forward, strafe, turn;
    private final ElapsedTime timer = new ElapsedTime();
    public void update (double rawForward, double rawStrafe, double rawTurn) {
        double dt = Math.min(timer.seconds(), 0.1);
        double step = MAX_RATE * dt;
        timer.reset();
//        forward = rawForward * Math.abs(rawForward) * dt;
//        strafe = rawStrafe * Math.abs(rawStrafe) * dt;
//        turn = rawTurn * Math.abs(rawTurn) * dt;
        forward += Range.clip(rawForward * Math.abs(rawForward) - forward, -step, step);
        strafe += Range.clip(rawStrafe * Math.abs(rawStrafe) - strafe, -step, step);
        turn += Range.clip(rawTurn * Math.abs(rawTurn) - turn, -step, step);
    }
    public void reset() {
        forward = strafe = turn = 0;
        timer.reset();
    }
    public double getForward() {
        return forward;
    }
    public double getStrafe() {
        return strafe;
    }
    public double getTurn() {
        return turn;
    }
    }
