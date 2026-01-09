package org.firstinspires.ftc.teamcode;

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

public class AutoSteps {
    ArrayList<Pose2d> Poses = new ArrayList<>();
    TrajectoryActionBuilder CurrentActionBuilder;
    Intake IntakeController;
    OutTake OutTakeController;
    DecoderWheel DecoderWheelController;
    String BaseColor = "Blue";

    public AutoSteps(TrajectoryActionBuilder ActionBuilder, Intake IntakeController, OutTake OutTakeController, DecoderWheel DecoderWheelController) {
        CurrentActionBuilder = ActionBuilder;
        this.IntakeController = IntakeController;
        this.OutTakeController = OutTakeController;
        this.DecoderWheelController = DecoderWheelController;
    }

    public Action BuildAndGetActionBuilder(String AllianceColor) {
        boolean ShouldFlip = !AllianceColor.equals(BaseColor);

        Poses.add(new Pose2d(52, 24, Math.toRadians(45)));
        Poses.add(new Pose2d(48.5, 8, Math.toRadians(-90)));
        Poses.add(new Pose2d(48.5, -1, Math.toRadians(-90)));
        Poses.add(new Pose2d(48.5, -6, Math.toRadians(-90)));
        Poses.add(new Pose2d(48.5, -11, Math.toRadians(-90)));
        Poses.add(new Pose2d(52, 24, Math.toRadians(45)));
        Poses.add(new Pose2d(10, 0, Math.toRadians(0)));
        Poses.add(new Pose2d(0, 0, Math.toRadians(0)));

        if (ShouldFlip) {Flip();}

        // Assumes all ball slots are empty
        double IntakeBallStepsTimeToWait = 0.1;
        SequentialAction IntakeBallsStep1 = new SequentialAction(
                new InstantAction(() -> IntakeController.SetPower(1)),
                new InstantAction(() -> IntakeController.ServosToIntake()),
                new InstantAction(() -> DecoderWheelController.IntakeModeOn()),
                new SleepAction(IntakeBallStepsTimeToWait)
        );
        SequentialAction IntakeBallsStep2 = new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> DecoderWheelController.RevolveRight()),
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToIntake())
        );
        SequentialAction IntakeBallsStep3 = new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToIntake())
        );
        SequentialAction IntakeBallsStep4 = new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> DecoderWheelController.RevolveRight()),
                new InstantAction(() -> DecoderWheelController.IntakeModeOff()),
                new InstantAction(() -> IntakeController.SetPower(0)),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new SleepAction(IntakeBallStepsTimeToWait)
        );

        VelConstraint SlowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(45))
        ));

        return CurrentActionBuilder
            .splineToLinearHeading(Poses.get(0), 0)
            .stopAndAdd(ShootAllBalls())

            // Intake first line of balls
            .splineToLinearHeading(Poses.get(1), 0)
            .stopAndAdd(IntakeBallsStep1)
            .strafeTo(new Vector2d(Poses.get(2).position.x, Poses.get(2).position.y), SlowVel)
            .stopAndAdd(IntakeBallsStep2)
            .strafeTo(new Vector2d(Poses.get(3).position.x, Poses.get(3).position.y), SlowVel)
            .stopAndAdd(IntakeBallsStep3)
            .strafeTo(new Vector2d(Poses.get(4).position.x, Poses.get(4).position.y), SlowVel)
            .stopAndAdd(IntakeBallsStep4)

            // Move to goal, then shoot all the collected balls
            .splineToLinearHeading(Poses.get(5), Math.toRadians(ShouldFlip ? -90 : 90))
            .stopAndAdd(ShootAllBalls())

            // Return to start for now
            .splineToLinearHeading(Poses.get(6), Math.toRadians(ShouldFlip ? -180 : 180))
            .strafeTo(new Vector2d(Poses.get(7).position.x, Poses.get(7).position.y))

            // Stop
            .stopAndAdd(new InstantAction(() -> this.OutTakeController.Stop()))
            .stopAndAdd(new InstantAction(() -> this.IntakeController.Stop()))

            .build();
    }

    public void Flip() {
        for (int i = 0; i < Poses.size(); i++) {
            Pose2d Pose = Poses.get(i);

            Pose2d NewPose = new Pose2d(
                Pose.position.x,
                -Pose.position.y,
                -Pose.heading.log()
            );

            Poses.set(i, NewPose);
        }
    }

    private SequentialAction ShootAllBalls() {
        double RampUpTime = 2.5;
        double ServoWaitTime = 0.5;
        double RevolveTime = 0.4;
        return new SequentialAction(
            new InstantAction(() -> IntakeController.SetPower(1)),
            new InstantAction(() -> IntakeController.ServosToNeutral()),
            new InstantAction(() -> OutTakeController.SetVelocity(1450 / 6000.0)),
            new AwaitAction(() -> OutTakeController.IsAtVelocity()),
            new InstantAction(() -> OutTakeController.ServosUp()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.ServosDown()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> DecoderWheelController.RevolveRight()),
            new WaitOneFrameAction(),
            new AwaitAction(() -> DecoderWheelController.IsAtTarget()),
            new AwaitAction(() -> OutTakeController.IsAtVelocity()),
            new InstantAction(() -> OutTakeController.ServosUp()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.ServosDown()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> DecoderWheelController.RevolveRight()),
            new WaitOneFrameAction(),
            new AwaitAction(() -> DecoderWheelController.IsAtTarget()),
            new AwaitAction(() -> OutTakeController.IsAtVelocity()),
            new InstantAction(() -> OutTakeController.ServosUp()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.ServosDown()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.SetVelocity(0)),
            new InstantAction(() -> IntakeController.SetPower(0))
        );
    }
}
