package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class AutoTest1 extends LinearOpMode {
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

        DcMotor FlMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor FrMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor BlMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor BrMotor = hardwareMap.get(DcMotor.class, "br");

        IMU Imu = hardwareMap.get(IMU.class, "imu");

        // init DiveController for controller drive
        this.DriveController = new Drive();
        this.DriveController.Init(FlMotor, FrMotor, BlMotor, BrMotor, gamepad1, gamepad2, Imu);
        this.DriveController.SetDriveMode(Drive.DriveMode.MANUAL);

        double LastRecTime = currentTimeMillis();
        while (opModeIsActive()) {
            long CurrTime = currentTimeMillis();
            double DeltaTime = (CurrTime / 1000.0) - LastRecTime;
            LastRecTime = CurrTime / 1000.0;

            if (DeltaTime < 0) {
                DeltaTime = 0;
            }

            AutoStepTimer -= DeltaTime;
            if (AutoStepTimer < 0) {
                AutoStep += 1;

                if (AutoStep == 0) {
                    AutoStepTimer = 1;
                }
                if (AutoStep == 1) {
                    break;
                }
            }

            if (AutoStep == 0) {
                Vector2 MoveDir = new Vector2(0, 0.25);
                this.DriveController.MoveInLocalDirectionAndTurn(MoveDir.GetNormal(), MoveDir.GetMagnitude(), 0, 1);
            }

            this.DriveController.Update(DeltaTime);
        }
    }
}
