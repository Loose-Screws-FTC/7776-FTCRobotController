package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class AutoTest2 extends LinearOpMode {
//    private Drive DriveController;
    private Intake IntakeController;

    private int AutoStep = -1;
    private double AutoStepTimer = 0;

    @Override
    public void runOpMode() {
        Drive.telemetry = telemetry;

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

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(new SequentialAction(
                    this.IntakeController.AutoStartIntaking(),
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(24, 0))
                            .build(),
                    this.IntakeController.AutoStopIntaking(),
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(0, 24))
                            .strafeTo(new Vector2d(0, 0))
                            .build()
            ));
        } else {
            throw new RuntimeException();
        }
    }
}
