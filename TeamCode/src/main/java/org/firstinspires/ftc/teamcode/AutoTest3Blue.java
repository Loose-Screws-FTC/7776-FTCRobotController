package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class AutoTest3Blue extends LinearOpMode {
    private Intake IntakeController;
    private DecoderWheel DecoderWheelController;
    private OutTake OutTakeController;

    @Override
    public void runOpMode() {
        Globals.telemetry = telemetry;

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        // Initialize hardware
        Servo InLeftServo = hardwareMap.get(Servo.class, "intakelefts");
        Servo InRightServo = hardwareMap.get(Servo.class, "intakerights");
        DcMotor InMotor = hardwareMap.get(DcMotor.class, "intake");
        this.IntakeController = new Intake();
        this.IntakeController.Init(InLeftServo, InRightServo, InMotor);
        DcMotor DecoderWheelMotor = hardwareMap.get(DcMotor.class, "ringdrive");
        this.DecoderWheelController = new DecoderWheel();
        this.DecoderWheelController.Init(DecoderWheelMotor);
        DcMotorEx OutLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "outr");
        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");
        Servo TiltServo = hardwareMap.get(Servo.class, "tiltservo");
        this.OutTakeController = new OutTake();
        this.OutTakeController.Init(OutLeft, OutRight, OutLeftServo, OutRightServo, TiltServo);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            // Assumes all ball slots are empty
            double IntakeBallStepsTimeToWait = 0.1;
            SequentialAction IntakeBallsStep1 = new SequentialAction(
                new InstantAction(() -> this.IntakeController.SetPower(1)),
                new InstantAction(() -> this.IntakeController.ServosToIntake()),
                new InstantAction(() -> this.DecoderWheelController.IntakeModeOn()),
                new SleepAction(IntakeBallStepsTimeToWait)
            );
            SequentialAction IntakeBallsStep2 = new SequentialAction(
                new SleepAction(IntakeBallStepsTimeToWait),
                new InstantAction(() -> IntakeController.ServosToNeutral()),
                new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
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
                new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
                new InstantAction(() -> this.DecoderWheelController.IntakeModeOff()),
                new InstantAction(() -> this.IntakeController.SetPower(0)),
                new InstantAction(() -> this.IntakeController.ServosToNeutral()),
                new SleepAction(IntakeBallStepsTimeToWait)
            );

            VelConstraint SlowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0),
                new AngularVelConstraint(Math.toRadians(45))
            ));

            Pose2d beginPose = new Pose2d(0, 0, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Action MainActionChain = drive.actionBuilder(beginPose)
                // vvv Put moves and actions here vvv

                // Move to goal, then shoot all the preloaded balls
//                .strafeTo(new Vector2d(31, 9))
//                .turnTo(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(31, 9, Math.toRadians(45)), 0)
                .stopAndAdd(ShootAllBalls())

                // Intake first line of balls
//                .strafeTo(new Vector2d(51.5, 10))
//                .turnTo(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(48.5, 8, Math.toRadians(-90)), 0)
                .stopAndAdd(IntakeBallsStep1)
                .strafeTo(new Vector2d(48.5, -1), SlowVel)
                .stopAndAdd(IntakeBallsStep2)
                .strafeTo(new Vector2d(48.5, -6), SlowVel)
                .stopAndAdd(IntakeBallsStep3)
                .strafeTo(new Vector2d(48.5, -11), SlowVel)
                .stopAndAdd(IntakeBallsStep4)

                // Move to goal, then shoot all the collected balls
//                .strafeTo(new Vector2d(31, 9))
//                .turnTo(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(31, 9, Math.toRadians(45)), Math.toRadians(90))
                .stopAndAdd(ShootAllBalls())

                // Return to start for now
                .splineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)), Math.toRadians(180))
                .strafeTo(new Vector2d(0, 0))

                // Stop
                .stopAndAdd(new InstantAction(() -> this.OutTakeController.Stop()))
                .stopAndAdd(new InstantAction(() -> this.IntakeController.Stop()))

                .build();

            // Start the auto
            waitForStart();
            Actions.runBlocking(
                new ParallelAction(
                    new UpdateAction(this.DecoderWheelController::Update),
                    new UpdateAction(this.IntakeController::Update),
                    new SequentialAction(
                        MainActionChain
                    )
                )
            );
        } else {
            throw new RuntimeException();
        }
    }

    private SequentialAction ShootAllBalls() {
        double RampUpTime = 2.5;
        double ServoWaitTime = 0.3;
        double RevolveTime = 0.4;
        return new SequentialAction(
            new InstantAction(() -> IntakeController.SetPower(1)),
            new InstantAction(() -> IntakeController.ServosToNeutral()),
            new InstantAction(() -> OutTakeController.SetVelocity(1550 / 6000.0)),
            new SleepAction(RampUpTime),
            new InstantAction(() -> OutTakeController.ServosUp()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.ServosDown()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> DecoderWheelController.RevolveRight()),
            new SleepAction(RevolveTime),
            new InstantAction(() -> OutTakeController.ServosUp()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.ServosDown()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> DecoderWheelController.RevolveRight()),
            new SleepAction(RevolveTime),
            new InstantAction(() -> OutTakeController.ServosUp()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> OutTakeController.ServosDown()),
            new SleepAction(ServoWaitTime),
            new InstantAction(() -> DecoderWheelController.RevolveRight()),
            new SleepAction(RevolveTime),
            new InstantAction(() -> OutTakeController.SetVelocity(0)),
            new InstantAction(() -> IntakeController.SetPower(0))
        );
    }
}
