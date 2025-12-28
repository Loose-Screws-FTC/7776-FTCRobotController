package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.List;

@Autonomous
public class AutoTest3Blue extends LinearOpMode {
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

            VelConstraint SlowVel = new MinVelConstraint(List.of(
                    drive.kinematics.new WheelVelConstraint(10.0),
                    new AngularVelConstraint(Math.toRadians(90))
            ));

            AccelConstraint SlowAccel = new ProfileAccelConstraint(-20.0, 20.0);

            Actions.runBlocking(
                new ParallelAction(
                    new UpdateAction(this.DecoderWheelController::Update),
                    new UpdateAction(this.IntakeController::Update),
                    new SequentialAction(
                        drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(40, 24), SlowVel, SlowAccel)
                            .turn(40 * Math.PI / 180)
                            .build(),

                        new InstantAction(() -> this.IntakeController.SetPower(1)),
//                        new InstantAction(() -> this.IntakeController.ServosToNeutral()),
//                        new InstantAction(() -> this.OutTakeController.SetVelocity(1550 / 6000.0)),
//                        new SleepAction(3),
//                        new InstantAction(() -> this.OutTakeController.ServosUp()),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> this.OutTakeController.ServosDown()),
//                        new SleepAction(1),
//                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
//                        new SleepAction(1),
//                        new InstantAction(() -> this.OutTakeController.ServosUp()),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> this.OutTakeController.ServosDown()),
//                        new SleepAction(1),
//                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
//                        new SleepAction(1),
//                        new InstantAction(() -> this.OutTakeController.ServosUp()),
//                        new SleepAction(0.5),
//                        new InstantAction(() -> this.OutTakeController.ServosDown()),
//                        new SleepAction(1),
//                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),
//                        new SleepAction(1),
//                        new InstantAction(() -> this.OutTakeController.SetVelocity(0)),
//                        new InstantAction(() -> this.IntakeController.SetPower(0)),

                        drive.actionBuilder()
                            .strafeTo(new Vector2d(50, 12), SlowVel, SlowAccel)
                            .turnTo(-90 * Math.PI / 180)
                            .build(),

//                        new InstantAction(() -> this.IntakeController.SetPower(1)),
//                        new InstantAction(() -> this.IntakeController.ServosToIntake()),
//                        new InstantAction(() -> this.DecoderWheelController.IntakeModeOn()),

                        drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(50, 0), SlowVel, SlowAccel)
                            .build(),

//                        new InstantAction(() -> this.DecoderWheelController.RevolveRight()),

                        drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(50, -5), SlowVel, SlowAccel)
                            .build(),

                        drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(0, 0), SlowVel, SlowAccel)
                            .build(),

                        drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(0, 0), SlowVel, SlowAccel)
                            .build()
                    )
                )
            );


        } else {
            throw new RuntimeException();
        }
    }
}
