package org.firstinspires.ftc.teamcode;// package org.firstinspires.ftc.teamcode.Decode.teleop;
// 
// import static java.lang.System.currentTimeMillis;
// 
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// 
// @Autonomous
// public class AutoTest0 extends LinearOpMode {
//     private ClassicDrive DriveController;
// 
//     @Override
//     public void runOpMode() {
//         ClassicDrive.telemetry = telemetry;
// 
//         waitForStart();
//         if (!opModeIsActive()) {
//             return;
//         }
// 
//         DriveController = new ClassicDrive();
//         DcMotor FlMotor = hardwareMap.get(DcMotor.class, "fl");
//         DcMotor FrMotor = hardwareMap.get(DcMotor.class, "fr");
//         DcMotor BlMotor = hardwareMap.get(DcMotor.class, "bl");
//         DcMotor BrMotor = hardwareMap.get(DcMotor.class, "br");
// 
//         // init DiveController for controller drive
//         this.DriveController = new ClassicDrive();
//         this.DriveController.Init(FlMotor, FrMotor, BlMotor, BrMotor, gamepad1, gamepad2);
//         this.DriveController.SetDriveMode(ClassicDrive.DriveMode.AUTO);
// 
//         double LastRecTime = currentTimeMillis();
//         while (opModeIsActive()) {
//             long CurrTime = currentTimeMillis();
//             double DeltaTime = (CurrTime / 1000.0) - LastRecTime;
//             LastRecTime = CurrTime / 1000.0;
// 
//             if (DeltaTime < 0) {
//                 DeltaTime = 0;
//             }
// 
//             DriveController.Update(DeltaTime);
//         }
//     }
// }
// 