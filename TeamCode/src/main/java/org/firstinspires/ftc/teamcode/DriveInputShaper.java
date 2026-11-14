package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveInputShaper{
    private double forward, strafe, turn;
    private final ElapsedTime timer = new ElapsedTime();
    public void update (double rawForward, double rawStrafe, double rawTurn) {
        double dt = Math.min(timer.seconds(), 0.1);
        timer.reset();
//        forward = rawForward * Math.abs(rawForward) * dt;
//        strafe = rawStrafe * Math.abs(rawStrafe) * dt;
//        turn = rawTurn * Math.abs(rawTurn) * dt;
        forward = rawForward * Math.abs(rawForward);
        strafe = rawStrafe * Math.abs(rawStrafe);
        turn = rawTurn * Math.abs(rawTurn);

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
