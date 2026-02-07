package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Config
public abstract class AutoSteps {
    public static boolean WithoutActionTelemetry = true;
    public static double TerminateTime = 30;
    boolean ShouldFlip;
    MecanumDrive Drive;
    RobotAbstractor Robot;

    public AutoSteps(MecanumDrive Drive, RobotAbstractor robot) {
        this.Drive = Drive;
        this.Robot = robot;
    }

    public abstract void Init();

    public abstract Action BuildAndGetActionBuilder();

    void TelemetryUpdate(double deltaTime) {
        Globals.telemetry.update();
    }

    public Pose2d FlipPose(Pose2d Pose) {
        return new Pose2d(
            Pose.position.x,
            -Pose.position.y,
            FlipAngle(Pose.heading.log())
        );
    }

    public double FlipAngle(double angle) {
        return -angle;
    }

    public Pose2d MapPose(Pose2d Pose) {
        if (ShouldFlip) {
            return FlipPose(Pose);
        } else {
            return Pose;
        }
    }

    public double MapAngle(double angle) {
        if (ShouldFlip) {
            return FlipAngle(angle);
        } else {
            return angle;
        }
    }

    public void FastRunAction(Action action) {
        if (WithoutActionTelemetry) {
            boolean b = true;
            while (b && !Thread.currentThread().isInterrupted()
                && Robot.Runtime.seconds() < TerminateTime) {
                TelemetryPacket packet = new TelemetryPacket();
                b = action.run(packet);
            }
        } else {
            Actions.runBlocking(action);
        }
    }
}
