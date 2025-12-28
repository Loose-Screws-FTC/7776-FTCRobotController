package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class AutoTest3Red extends LinearOpMode {
//    private Drive DriveController;
    private Intake IntakeController;
    private DecoderWheel DecoderWheelController;
    private OutTake OutTakeController;

    private int AutoStep = -1;
    private double AutoStepTimer = 0;

    @Override
    public void runOpMode() {
        Globals.telemetry = telemetry;

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

//        DcMotor FlMotor = hardwareMap.get(DcMotor.class, "fl");
//        DcMotor FrMotor = hardwareMap.get(DcMotor.class, "fr");
//        DcMotor BlMotor = hardwareMap.get(DcMotor.class, "bl");
//        DcMotor BrMotor = hardwareMap.get(DcMotor.class, "br");
//
//        IMU Imu = hardwareMap.get(IMU.class, "imu");
//
//        // init DiveController for controller drive
//        this.DriveController = new Drive();
//        this.DriveController.Init(FlMotor, FrMotor, BlMotor, BrMotor, gamepad1, gamepad2, Imu);
//        this.DriveController.SetDriveMode(Drive.DriveMode.MANUAL);

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

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

//            Actions.runBlocking(
//                new ParallelAction(
//                    new UpdateAction(this.DecoderWheelController::Update),
//                    new SequentialAction(
//                        new InstantAction(() -> this.DecoderWheelController.IntakeModeOn()),
//                        new InstantAction(() -> this.IntakeController.SetPower(1)),
//                        new InstantAction(() -> this.IntakeController.ServosToIntake()),
//                        drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(28, 0), 0)
//                            .build(),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> this.DecoderWheelController.RevolveLeft()),
//                        new InstantAction(() -> this.DecoderWheelController.IntakeModeOff()),
//                        new InstantAction(() -> this.IntakeController.SetPower(1)),
//                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
//                        new InstantAction(() -> this.DecoderWheelController.IntakeModeOn()),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> this.IntakeController.SetPower(1)),
//                        new InstantAction(() -> this.IntakeController.ServosToIntake()),
//                        drive.actionBuilder(beginPose.plus(new Twist2d(new Vector2d(28, 0), 0)))
//                            .turn(Math.PI / 2)
//                            .lineToY(24)
//                            .build(),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> this.IntakeController.SetPower(0)),
//                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
//                        new InstantAction(() -> this.DecoderWheelController.IntakeModeOff()),
//                        drive.actionBuilder(beginPose)
//                            .splineToSplineHeading(new Pose2d(0.1, 0.1, 0), 0)
//                            .build(),
//                        new InstantAction(() -> this.OutTakeController.SetVelocity(2000)),
//                        new InstantAction(() -> this.IntakeController.SetPower(0)),
//                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
//                        new SleepAction(4),
//                        new InstantAction(() -> this.OutTakeController.ServosUp()),
//                        new SleepAction(0.2),
//                        new InstantAction(() -> this.OutTakeController.ServosDown()),
//                        new SleepAction(0.2),
//                        new InstantAction(() -> this.DecoderWheelController.RevolveLeft()),
//                        new InstantAction(() -> this.IntakeController.SetPower(1)),
//                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
//                        new SleepAction(0.6),
//                        new InstantAction(() -> this.IntakeController.SetPower(0)),
//                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
//                        new InstantAction(() -> this.OutTakeController.ServosUp()),
//                        new SleepAction(0.2),
//                        new InstantAction(() -> this.OutTakeController.ServosDown()),
//                        new InstantAction(() -> this.OutTakeController.SetVelocity(0))
//                    )
//                )
//            );

            Actions.runBlocking(
                new ParallelAction(
                    new UpdateAction(this.DecoderWheelController::Update),
                    new UpdateAction(this.IntakeController::Update),
                    new SequentialAction(
                        drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(67.8822509939, 0))
                            .build(),

                        new InstantAction(() -> this.IntakeController.SetPower(1)),
                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
                        new InstantAction(() -> this.OutTakeController.SetVelocity(1550 / 6000.0)),
                        new SleepAction(3),
                        new InstantAction(() -> this.OutTakeController.ServosUp()),
                        new SleepAction(0.5),
                        new InstantAction(() -> this.OutTakeController.ServosDown()),
                        new SleepAction(0.5),
                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
                        new SleepAction(1),
                        new InstantAction(() -> this.OutTakeController.ServosUp()),
                        new SleepAction(0.5),
                        new InstantAction(() -> this.OutTakeController.ServosDown()),
                        new SleepAction(0.5),
                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
                        new SleepAction(1),
                        new InstantAction(() -> this.OutTakeController.ServosUp()),
                        new SleepAction(0.5),
                        new InstantAction(() -> this.OutTakeController.ServosDown()),
                        new SleepAction(0.5),
                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
                        new SleepAction(1),
                        new InstantAction(() -> this.OutTakeController.SetVelocity(0)),
                        new InstantAction(() -> this.IntakeController.SetPower(0)),

                        drive.actionBuilder(beginPose.plus(new Twist2d(new Vector2d(67.8822509939, 0), 0)))
                            .strafeTo(new Vector2d(67.8822509939, 0))
                            .turn(45 * Math.PI / 180)
                            .build()
                    )
                )
            );


        } else {
            throw new RuntimeException();
        }
    }
}
