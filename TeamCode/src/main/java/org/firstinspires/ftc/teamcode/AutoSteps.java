package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

@Config
public class AutoSteps {
    ArrayList<Pose2d> Poses = new ArrayList<>();
    TrajectoryActionBuilder CurrentActionBuilder;
    MecanumDrive Drive;
    RobotAbstractor Robot;
    Intake IntakeController;
    OutTake OutTakeController;
    DecoderWheel DecoderWheelController;
    TeamColor BaseColor = TeamColor.BLUE;
    TeamColor AllianceColor;
    boolean ShouldFlip;

    public static double LaunchRPM = 1650;

    public enum TeamColor {
        BLUE,
        RED
    }

    public AutoSteps(TrajectoryActionBuilder ActionBuilder, TeamColor AllianceColor, MecanumDrive Drive, RobotAbstractor robot) {
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
                DecoderWheel.BallColor.GREEN,
                DecoderWheel.BallColor.PURPLE,
                DecoderWheel.BallColor.PURPLE
        );
    }

    public Action BuildAndGetActionBuilder() {
        Poses.add(new Pose2d(52, 24, Math.toRadians(45)));
        Poses.add(new Pose2d(48.5, 8, Math.toRadians(-90)));
        Poses.add(new Pose2d(48.5, -1, Math.toRadians(-90)));
        Poses.add(new Pose2d(48.5, -6, Math.toRadians(-90)));
        Poses.add(new Pose2d(48.5, -11, Math.toRadians(-90)));
        Poses.add(new Pose2d(52, 24, Math.toRadians(45)));
        Poses.add(new Pose2d(10, 0, Math.toRadians(0)));
        Poses.add(new Pose2d(0, 0, Math.toRadians(0)));

        Poses.add(new Pose2d(30, 38, Math.toRadians(0)));

        if (ShouldFlip) {Flip();}

        double RampUpTime = 2.5;
        double ServoWaitTime = 1;
        double RevolveTime = 0.4;
        double RPMStabilizeTime = 0.2;
        Supplier<SequentialAction> ShootAllBalls = () -> new SequentialAction(
                new InstantAction(() -> IntakeController.SetPower(1)),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new SleepAction(0.1), // wait for the intake servos to move
                new InstantAction(() -> DecoderWheelController.RevolveToColor(BallOrder.GameOrder.Ball1)),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new SleepAction(RPMStabilizeTime),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new InstantAction(() -> Robot.OutTakeBall()),
                new SleepAction(ServoWaitTime),
                new InstantAction(() -> OutTakeController.ServosDown()),
                new SleepAction(ServoWaitTime),
                new InstantAction(() -> DecoderWheelController.RevolveToColor(BallOrder.GameOrder.Ball2)),
                new WaitOneFrameAction(),
                new AwaitAction(() -> DecoderWheelController.IsAtTarget()),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new SleepAction(RPMStabilizeTime),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new InstantAction(() -> Robot.OutTakeBall()),
                new SleepAction(ServoWaitTime),
                new InstantAction(() -> OutTakeController.ServosDown()),
                new SleepAction(ServoWaitTime),
                new InstantAction(() -> DecoderWheelController.RevolveToColor(BallOrder.GameOrder.Ball3)),
                new WaitOneFrameAction(),
                new AwaitAction(() -> DecoderWheelController.IsAtTarget()),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new SleepAction(RPMStabilizeTime),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new InstantAction(() -> Robot.OutTakeBall()),
                new SleepAction(ServoWaitTime),
                new InstantAction(() -> OutTakeController.ServosDown()),
                new SleepAction(ServoWaitTime),
                new InstantAction(() -> IntakeController.SetPower(0))
        );

        // Assumes all ball slots are empty
        double IntakeBallStepsTimeToWait = 0.1;
        Supplier<SequentialAction> IntakeBallsStep1 = () -> new SequentialAction(
                new InstantAction(() -> IntakeController.SetPower(1)),
                new InstantAction(() -> IntakeController.ServosToIntake()),
                new InstantAction(() -> DecoderWheelController.IntakeModeOn()),
                new SleepAction(IntakeBallStepsTimeToWait),
                new CollectBallAction(Drive, 3)
        );
        Supplier<SequentialAction> IntakeBallsStep2 = () -> new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> DecoderWheelController.RevolveRight()),
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToIntake()),
                new CollectBallAction(Drive, 3)
        );
        Supplier<SequentialAction> IntakeBallsStep3 = () -> new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToIntake()),
                new CollectBallAction(Drive, 3)
        );
        Supplier<SequentialAction> IntakeBallsStep4 = () -> new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> DecoderWheelController.RevolveRight()),
                new InstantAction(() -> DecoderWheelController.IntakeModeOff()),
                new InstantAction(() -> IntakeController.SetPower(0)),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new SleepAction(IntakeBallStepsTimeToWait)
        );

        VelConstraint SlowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(4.0),
                new AngularVelConstraint(Math.toRadians(45))
        ));

        return CurrentActionBuilder
            .stopAndAdd(() -> OutTakeController.SetVelocity(LaunchRPM / 6000.0))
            .splineToLinearHeading(Poses.get(8), 0)
            .stopAndAdd(() -> BallOrder.DetectBallOrder(this.Robot))

            .splineToLinearHeading(Poses.get(0), 0)
            .stopAndAdd(ShootAllBalls.get())

            // Intake first line of balls
            .splineToLinearHeading(Poses.get(1), 0)
            .stopAndAdd(IntakeBallsStep1.get())
            .strafeTo(new Vector2d(Poses.get(2).position.x, Poses.get(2).position.y), SlowVel)
            .stopAndAdd(IntakeBallsStep2.get())
            .strafeTo(new Vector2d(Poses.get(3).position.x, Poses.get(3).position.y), SlowVel)
            .stopAndAdd(IntakeBallsStep3.get())
            .strafeTo(new Vector2d(Poses.get(4).position.x, Poses.get(4).position.y), SlowVel)
            .stopAndAdd(IntakeBallsStep4.get())
            .stopAndAdd(() -> DecoderWheelController.SetCurrentColors(
                    DecoderWheel.BallColor.PURPLE,
                    DecoderWheel.BallColor.PURPLE,
                    DecoderWheel.BallColor.GREEN
            ))

            // Move to goal, then shoot all the collected balls
            .splineToLinearHeading(Poses.get(5), Math.toRadians(ShouldFlip ? -90 : 90))
            .stopAndAdd(ShootAllBalls.get())

            // Return to start for now
            .splineToLinearHeading(Poses.get(6), Math.toRadians(ShouldFlip ? -180 : 180))
            .strafeTo(new Vector2d(Poses.get(7).position.x, Poses.get(7).position.y))

            // Stop
            .stopAndAdd(new InstantAction(() -> this.OutTakeController.Stop()))
            .stopAndAdd(new InstantAction(() -> this.IntakeController.Stop()))

            .build();
    }

    public Pose2d FlipPose(Pose2d Pose) {
        return new Pose2d(
            Pose.position.x,
            -Pose.position.y,
            -Pose.heading.log()
        );
    }

    public void Flip() {
        Poses.replaceAll(this::FlipPose);
    }
}
