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
        Intake intakeController = new Intake();
        intakeController.Init(InLeftServo, InRightServo, InMotor);

        DcMotor DecoderWheelMotor = hardwareMap.get(DcMotor.class, "ringdrive");
        DecoderWheel decoderWheelController = new DecoderWheel();
        decoderWheelController.Init(DecoderWheelMotor);

        DcMotorEx OutLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "outr");
        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");
        Servo TiltServo = hardwareMap.get(Servo.class, "tiltservo");
        OutTake outTakeController = new OutTake();
        outTakeController.Init(OutLeft, OutRight, OutLeftServo, OutRightServo, TiltServo);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            Pose2d beginPose = new Pose2d(1, -7, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            TrajectoryActionBuilder MainActionBuilder = drive.actionBuilder(beginPose);

            AutoSteps StepsBuilder = new AutoSteps(MainActionBuilder, intakeController, outTakeController, decoderWheelController);

            Action MainActionChain = StepsBuilder.BuildAndGetActionBuilder("Blue");

            // Start the auto
            waitForStart();
            Actions.runBlocking(
                    new ParallelAction(
                            new UpdateAction(decoderWheelController::Update),
                            new UpdateAction(intakeController::Update),
                            new UpdateAction(outTakeController::Update),
                            new UpdateAction(this::TelemetryUpdate),
                            new SequentialAction(
                                    MainActionChain
                            )
                    )
            );
        } else {
            throw new RuntimeException();
        }
    }

    public void TelemetryUpdate(double DeltaTime) {
        telemetry.addData("should flip", "no");
        telemetry.update();
    }
}