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
public class AutoSteps {
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
        double RampUpTime = 2.5;
        double ServoUpTime = 0.5;
        double ServoDownTime = 1;
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
                new SleepAction(ServoUpTime),
                new InstantAction(() -> OutTakeController.ServosDown()),
                new SleepAction(ServoDownTime),
                new InstantAction(() -> DecoderWheelController.RevolveToColor(BallOrder.GameOrder.Ball2)),
                new WaitOneFrameAction(),
                new AwaitAction(() -> DecoderWheelController.IsAtTarget()),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new SleepAction(RPMStabilizeTime),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new InstantAction(() -> Robot.OutTakeBall()),
                new SleepAction(ServoUpTime),
                new InstantAction(() -> OutTakeController.ServosDown()),
                new SleepAction(ServoDownTime),
                new InstantAction(() -> DecoderWheelController.RevolveToColor(BallOrder.GameOrder.Ball3)),
                new WaitOneFrameAction(),
                new AwaitAction(() -> DecoderWheelController.IsAtTarget()),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new SleepAction(RPMStabilizeTime),
                new AwaitAction(() -> OutTakeController.IsAtVelocity()),
                new InstantAction(() -> Robot.OutTakeBall()),
                new SleepAction(ServoUpTime),
                new InstantAction(() -> OutTakeController.ServosDown()),
                new SleepAction(ServoDownTime),
                new InstantAction(() -> IntakeController.SetPower(0))
        );

        // Assumes all ball slots are empty
        double IntakeBallStepsTimeToWait = 0.1;
        Supplier<SequentialAction> IntakeBallsStep = () -> new SequentialAction(
                new InstantAction(() -> Robot.ShouldIntake = true),
                new CollectBallAction(Drive, 28),
                new InstantAction(() -> Robot.ShouldIntake = false)
        );
//        Supplier<SequentialAction> IntakeBallsStep1 = () -> new SequentialAction(
//                new InstantAction(() -> IntakeController.SetPower(1)),
//                new InstantAction(() -> IntakeController.ServosToIntake()),
//                new InstantAction(() -> DecoderWheelController.IntakeModeOn()),
//                new SleepAction(IntakeBallStepsTimeToWait),
//                new CollectBallAction(Drive, 5),
//                new InstantAction(() -> DecoderWheelController.SetIntakedColor(DecoderWheel.BallColor.PURPLE))
//        );
//        Supplier<SequentialAction> IntakeBallsStep2 = () -> new SequentialAction(
//                new SleepAction(IntakeBallStepsTimeToWait),
//                new InstantAction(() -> IntakeController.ServosToNeutral()),
//                new InstantAction(() -> DecoderWheelController.RevolveRight()),
//                new SleepAction(IntakeBallStepsTimeToWait),
//                new InstantAction(() -> IntakeController.ServosToIntake()),
//                new CollectBallAction(Drive, 3),
//                new InstantAction(() -> DecoderWheelController.SetIntakedColor(DecoderWheel.BallColor.PURPLE))
//        );
//        Supplier<SequentialAction> IntakeBallsStep3 = () -> new SequentialAction(
//                new SleepAction(IntakeBallStepsTimeToWait),
//                new InstantAction(() -> IntakeController.ServosToNeutral()),
//                new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
//                new SleepAction(IntakeBallStepsTimeToWait),
//                new InstantAction(() -> IntakeController.ServosToIntake()),
//                new CollectBallAction(Drive, 5),
//                new InstantAction(() -> DecoderWheelController.SetIntakedColor(DecoderWheel.BallColor.GREEN))
//        );
//        Supplier<SequentialAction> IntakeBallsStep4 = () -> new SequentialAction(
//                new SleepAction(IntakeBallStepsTimeToWait),
//                new InstantAction(() -> IntakeController.ServosToNeutral()),
//                new InstantAction(() -> DecoderWheelController.RevolveRight()),
//                new InstantAction(() -> DecoderWheelController.IntakeModeOff()),
//                new InstantAction(() -> IntakeController.SetPower(0)),
//                new InstantAction(() -> IntakeController.ServosToNeutral()),
//                new SleepAction(IntakeBallStepsTimeToWait)
//        );

        VelConstraint SlowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(4.0),
                new AngularVelConstraint(Math.toRadians(45))
        ));

        Action steps =  CurrentActionBuilder
            .stopAndAdd(() -> OutTakeController.SetVelocity(LaunchRPM / 6000.0))
            .splineToLinearHeading(MapPose(new Pose2d(10, 18, Math.toRadians(ShouldFlip ? -45 : 45))), 0)
            .stopAndAdd(new FindBallOrderAction(Robot))
            .stopAndAdd(() -> DecoderWheelController.RevolveToColor(BallOrder.GameOrder.Ball1))

            .splineToLinearHeading(MapPose(new Pose2d(52, 24, Math.toRadians(45))), 0)
            .stopAndAdd(ShootAllBalls.get())

            // Intake first line of balls
            .splineToLinearHeading(MapPose(new Pose2d(48.5, 5, Math.toRadians(-90))), 0)
            .stopAndAdd(IntakeBallsStep.get())
//            .stopAndAdd(IntakeBallsStep1.get())
//            .strafeTo(MapPose(new Pose2d(48.5, -5, Math.toRadians(-90))).position, SlowVel)
//            .stopAndAdd(IntakeBallsStep2.get())
//            .strafeTo(MapPose(new Pose2d(48.5, -10, Math.toRadians(-90))).position, SlowVel)
//            .stopAndAdd(IntakeBallsStep3.get())
//            .strafeTo(MapPose(new Pose2d(48.5, -15, Math.toRadians(-90))).position, SlowVel)
//            .stopAndAdd(IntakeBallsStep4.get())

            // Move to goal, then shoot all the collected balls
            .splineToLinearHeading(MapPose(new Pose2d(52, 24, Math.toRadians(45))), Math.toRadians(ShouldFlip ? -90 : 90))
            .stopAndAdd(ShootAllBalls.get())

            // Return to start for now
            .splineToLinearHeading(MapPose(new Pose2d(10, 0, Math.toRadians(0))), Math.toRadians(ShouldFlip ? -90 : 90))
            .strafeTo(MapPose(new Pose2d(0, 0, Math.toRadians(0))).position)

            // Stop
            .stopAndAdd(new InstantAction(() -> this.OutTakeController.Stop()))
            .stopAndAdd(new InstantAction(() -> this.IntakeController.Stop()))

            .build();

        return new ParallelAction(
            new UpdateAction(DecoderWheelController::Update),
            new UpdateAction(IntakeController::Update),
            new UpdateAction(Robot::IntakeUpdate),
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
