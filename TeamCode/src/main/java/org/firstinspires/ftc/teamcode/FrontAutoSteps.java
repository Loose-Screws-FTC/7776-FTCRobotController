package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;
import java.util.function.Supplier;

@Config
public class FrontAutoSteps {
    TrajectoryActionBuilder CurrentActionBuilder;
    MecanumDrive Drive;
    RobotAbstractor Robot;
    Intake IntakeController;
    OutTake OutTakeController;
    DecoderWheel DecoderWheelController;
    TeamColor BaseColor = TeamColor.BLUE;
    TeamColor AllianceColor;
    boolean ShouldFlip;

    public static double LaunchRPM = 1520;

    public FrontAutoSteps(TrajectoryActionBuilder ActionBuilder, TeamColor AllianceColor, MecanumDrive Drive, RobotAbstractor robot) {
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
        // Assumes all ball slots are empty
        double IntakeBallStepsTimeToWait = 0.1;
        Supplier<SequentialAction> IntakeBallsStep = () -> new SequentialAction(
                new InstantAction(() -> Robot.ShouldIntake = true),
                new CollectBallAction(Drive, 26),
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

        Action steps = CurrentActionBuilder
            .stopAndAdd(() -> OutTakeController.SetVelocity(LaunchRPM / 6000.0))
            .stopAndAdd(() -> OutTakeController.SetIsFiring(true))
            .splineToLinearHeading(MapPose(new Pose2d(10, 18, Math.toRadians(-45))), 0)
            .stopAndAdd(new FindBallOrderAction(Robot, 0.5))
            .stopAndAdd(() -> Robot.RecordMotifOffset())
            .stopAndAdd(() -> DecoderWheelController.RevolveToColor(Robot.GetMotifBallColor(0)))

            .splineToLinearHeading(MapPose(new Pose2d(52, 24, Math.toRadians(45))), 0)
            .stopAndAdd(Robot.ShootAllBallsAction())

            // Intake first line of balls
            .splineToLinearHeading(MapPose(new Pose2d(48.5, 5, Math.toRadians(-90))), 0)
            .stopAndAdd(IntakeBallsStep.get())
            .stopAndAdd(() -> Robot.RecordMotifOffset())
            .stopAndAdd(() -> DecoderWheelController.RevolveToColor(Robot.GetMotifBallColor(0)))
//            .stopAndAdd(IntakeBallsStep1.get())
//            .strafeTo(MapPose(new Pose2d(48.5, -5, Math.toRadians(-90))).position, SlowVel)
//            .stopAndAdd(IntakeBallsStep2.get())
//            .strafeTo(MapPose(new Pose2d(48.5, -10, Math.toRadians(-90))).position, SlowVel)
//            .stopAndAdd(IntakeBallsStep3.get())
//            .strafeTo(MapPose(new Pose2d(48.5, -15, Math.toRadians(-90))).position, SlowVel)
//            .stopAndAdd(IntakeBallsStep4.get())

            // Move to goal, then shoot all the collected balls
            .splineToLinearHeading(MapPose(new Pose2d(52, 24, Math.toRadians(45))), Math.toRadians(ShouldFlip ? -90 : 90))
            .stopAndAdd(Robot.ShootAllBallsAction())

            // Intake second line of balls
            .splineToLinearHeading(MapPose(new Pose2d(72.5, 5, Math.toRadians(-90))), 0)
            .stopAndAdd(IntakeBallsStep.get())
            .stopAndAdd(() -> Robot.RecordMotifOffset())
            .stopAndAdd(() -> DecoderWheelController.RevolveToColor(Robot.GetMotifBallColor(0)))

            // Move to goal, then shoot all the collected balls
            .splineToLinearHeading(MapPose(new Pose2d(52, 24, Math.toRadians(45))), Math.toRadians(ShouldFlip ? -90 : 90))
            .stopAndAdd(Robot.ShootAllBallsAction())

            // Return to start for now
            .strafeToLinearHeading(
                MapPose(new Pose2d(10, 0, Math.toRadians(270))).position,
                MapPose(new Pose2d(10, 0, Math.toRadians(270))).heading.log()
            )
            .strafeTo(MapPose(new Pose2d(0, 0, Math.toRadians(270))).position)

            // Stop
            .stopAndAdd(new InstantAction(() -> this.OutTakeController.Stop()))
            .stopAndAdd(new InstantAction(() -> this.IntakeController.Stop()))

            .build();

        return new ParallelAction(
            new UpdateAction(Robot::Update),
            new UpdateAction(Robot::AutoIntakeUpdate),
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
