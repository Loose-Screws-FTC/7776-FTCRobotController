package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;
import java.util.function.Supplier;

@Config
public class BackAutoSteps {
    TrajectoryActionBuilder CurrentActionBuilder;
    MecanumDrive Drive;
    RobotAbstractor Robot;
    Intake IntakeController;
    OutTake OutTakeController;
    DecoderWheel DecoderWheelController;
    TeamColor BaseColor = TeamColor.RED;
    TeamColor AllianceColor;
    boolean ShouldFlip;

    public static double LaunchRPM = 1785;

    public static double FireTime = 20;

    public BackAutoSteps(TrajectoryActionBuilder ActionBuilder, TeamColor AllianceColor, MecanumDrive Drive, RobotAbstractor robot) {
        CurrentActionBuilder = ActionBuilder;
        this.AllianceColor = AllianceColor;
        this.ShouldFlip = BaseColor != AllianceColor;
        this.Drive = Drive;
        this.Robot = robot;
        this.IntakeController = robot.IntakeSys;
        this.OutTakeController = robot.OutTakeSys;
        this.DecoderWheelController = robot.DecoderWheelSys;
    }

    public void Init() {
        this.DecoderWheelController.SetCurrentColors(
            BallColor.GREEN,
            BallColor.PURPLE,
            BallColor.PURPLE
        );
    }

    public Action BuildAndGetActionBuilder() {
        // https://www.desmos.com/calculator/xrp25mvpxb
        Action steps = CurrentActionBuilder
            .stopAndAdd(() -> OutTakeController.SetVelocity(LaunchRPM / 6000.0))
            .stopAndAdd(() -> OutTakeController.SetIsFiring(true))
            .splineToLinearHeading(MapPose(new Pose2d(0, 5, Math.toRadians(0))), 0)
            .splineToLinearHeading(MapPose(new Pose2d(0, 5, Math.toRadians(70))), 0)
            .stopAndAdd(new AwaitAction(() -> Runtime.seconds() > FireTime))
            .stopAndAdd(Robot.ShootAllBallsAction())
            .splineToLinearHeading(MapPose(new Pose2d(20, 5, Math.toRadians(0))), 0)
            .build();

        return new ParallelAction(
            new UpdateAction(DecoderWheelController::Update),
            new UpdateAction(IntakeController::Update),
            new UpdateAction(Robot::AutoIntakeUpdate),
            new UpdateAction(OutTakeController::Update),
            new UpdateAction(this::TelemetryUpdate),
            new SequentialAction(
                steps
            )
        );
    }

    void TelemetryUpdate(double deltaTime) {
        Globals.telemetry.update();
    }

    public Pose2d FlipPose(Pose2d Pose) {
        return new Pose2d(
            Pose.position.x,
            -Pose.position.y,
            -Pose.heading.log()
        );
    }

    public Pose2d MapPose(Pose2d Pose) {
        if (ShouldFlip) {
            return FlipPose(Pose);
        } else {
            return Pose;
        }
    }
}
