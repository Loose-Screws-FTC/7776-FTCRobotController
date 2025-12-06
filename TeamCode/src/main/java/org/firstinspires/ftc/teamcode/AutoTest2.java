package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class AutoTest2 extends LinearOpMode {
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

        this.DecoderWheelController.SetIntake(IntakeController);

        DcMotorEx OutLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "outr");

        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");

        this.OutTakeController = new OutTake();
        this.OutTakeController.Init(OutLeft, OutRight, OutLeftServo, OutRightServo);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

//            Actions.runBlocking(new SequentialAction(
//                    drive.actionBuilder(beginPose)
//                            .strafeTo(new Vector2d(27.5, 0))
//                            .turnTo(90)
//                            .build(),
//                    this.IntakeController.AutoStartIntaking(),
//                    drive.actionBuilder(beginPose)
//                            .strafeTo(new Vector2d(27.5, 23))
//                            .turnTo(90)
//                            .build(),
//                    new SleepAction(0.5),
//                    this.DecoderWheelController.AutoRevolveRight(),
//                    new SleepAction(0.5),
//                    drive.actionBuilder(beginPose)
//                            .strafeTo(new Vector2d(27.5, 28))
//                            .turnTo(90)
//                            .build(),
//                    new SleepAction(0.5),
//                    this.DecoderWheelController.AutoRevolveRight(),
//                    new SleepAction(0.5),
//                    drive.actionBuilder(beginPose)
//                            .strafeTo(new Vector2d(27.5, 33))
//                            .turnTo(90)
//                            .build(),
//                    this.IntakeController.AutoStopIntaking()
//            ));

            Actions.runBlocking(new ParallelAction(
                    this.DecoderWheelController.AutoStartUpdateLoop(),
                    new SequentialAction(
                        this.DecoderWheelController.AutoIntakeModeOn(),
                        this.IntakeController.AutoStartIntaking(),
                        drive.actionBuilder(beginPose)
                                .splineToSplineHeading(new Pose2d(28, 24, Math.PI / 2), Math.PI / 2)
                                .build(),
                        new SleepAction(0.5),
                        this.IntakeController.AutoStopIntaking(),
                        this.DecoderWheelController.AutoIntakeModeOff(),
                        drive.actionBuilder(beginPose)
                                .splineToSplineHeading(new Pose2d(0.1, 0.1, 0), 0)
                                .build(),
                        this.DecoderWheelController.AutoRevolveLeft(),
                        this.IntakeController.AutoStartIntakingForRevolve(),
                        new SleepAction(0.6),
                        this.OutTakeController.AutoSpinUp(),
                        this.IntakeController.AutoStopIntaking(),
                        new SleepAction(4),
                        this.OutTakeController.AutoServosUp(),
                        new SleepAction(1),
                        this.OutTakeController.AutoSpinDown(),
                        this.OutTakeController.AutoServosDown()
            )));
        } else {
            throw new RuntimeException();
        }
    }
}
