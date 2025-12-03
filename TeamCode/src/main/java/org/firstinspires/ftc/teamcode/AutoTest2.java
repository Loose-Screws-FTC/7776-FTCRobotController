package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class AutoTest2 extends LinearOpMode {
    private Drive DriveController;

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

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(24, 0))
                            .strafeTo(new Vector2d(0, 24))
                            .strafeTo(new Vector2d(0, 0))
                            .build()
            );
        } else {
            throw new RuntimeException();
        }
    }
}
