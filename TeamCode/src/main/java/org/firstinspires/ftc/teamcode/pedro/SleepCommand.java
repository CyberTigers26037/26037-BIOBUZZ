package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.Collections;
import java.util.Set;

public class SleepCommand implements Command {
    private Timer timer;

    public SleepCommand(double seconds) {
        timer = new Timer(seconds);
    }

    @Override
    public Set<Object> requirements() {
        return Collections.emptySet();
    }

    @Override
    public int priority() {
        return 0;
    }

    @Override
    public InterruptedBehavior interruptedBehavior() {
        return null;
    }

    @Override
    public ConflictBehavior conflictBehavior() {
        return null;
    }

    @Override
    public BlockedBehavior blockedBehavior() {
        return null;
    }

    @Override
    public void start() {
        timer.start();
    }

    @Override
    public boolean done() {
        return timer.isExpired();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(EndCondition endCondition) {

    }
}
