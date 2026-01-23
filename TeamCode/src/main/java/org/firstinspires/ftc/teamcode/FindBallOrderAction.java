package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.robot.Robot;

@Config
public class FindBallOrderAction implements Action {
    public static double MaxWaitTime = 0.5;
    double StartTime = Double.NaN;
    RobotAbstractor Robot;

    public FindBallOrderAction(RobotAbstractor robot) {
        this.Robot = robot;
    }

    public boolean run(@NonNull TelemetryPacket packet) {
        double Now = currentTimeMillis() / 1000.0;
        if (Double.isNaN(StartTime)) StartTime = Now;
        if (Now > StartTime + MaxWaitTime) return false;
        BallOrder order = BallOrder.FindBallOrder(Robot);
        if (order == BallOrder.IDK) {
            return true;
        } else {
            BallOrder.GameOrder = order;
            return false;
        }
    }
}
