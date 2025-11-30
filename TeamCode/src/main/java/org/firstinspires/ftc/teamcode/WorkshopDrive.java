package org.firstinspires.ftc.teamcode;// /* Copyright (c) 2017 FIRST. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted (subject to the limitations in the disclaimer below) provided that
//  * the following conditions are met:
//  *
//  * Redistributions of source code must retain the above copyright notice, this list
//  * of conditions and the following disclaimer.
//  *
//  * Redistributions in binary form must reproduce the above copyright notice, this
//  * list of conditions and the following disclaimer in the documentation and/or
//  * other materials provided with the distribution.
//  *
//  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//  * promote products derived from this software without specific prior written permission.
//  *
//  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */
// 
// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
// 
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// 
// 
// /*
//  * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
//  * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
//  * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
//  * class is instantiated on the Robot Controller and executed.
//  *
//  * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
//  * It includes all the skeletal structure that all linear OpModes contain.
//  *
//  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
//  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
//  */
// 
// @TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
// 
// public class WorkshopDrive extends LinearOpMode {
// 
//     // Declare OpMode members.
//     private ElapsedTime runtime = new ElapsedTime();
//     private DcMotor m0 = null;
//     private DcMotor m1 = null;
//     private DcMotor m2 = null;
//     private DcMotor m3 = null;
//     private IMU imu = null;
//     double startOrientation;
// 
//     @Override
//     public void runOpMode() {
//         telemetry.addData("Status", "Initialized");
//         telemetry.update();
// 
//         // Initialize the hardware variables. Note that the strings used here as parameters
//         // to 'get' must correspond to the names assigned during the robot configuration
//         // step (using the FTC Robot Controller app on the phone).
//         m0  = hardwareMap.get(DcMotor.class, "0");
//         m1 = hardwareMap.get(DcMotor.class, "1");
//         m2 = hardwareMap.get(DcMotor.class,"2");
//         m3 = hardwareMap.get(DcMotor.class,"3");
//         imu = hardwareMap.get(IMU.class, "imu");
// 
//         startOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//         // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//         m0.setDirection(DcMotor.Direction.REVERSE);
//         m1.setDirection(DcMotor.Direction.REVERSE);
//         m2.setDirection(DcMotor.Direction.REVERSE);
//         m3.setDirection(DcMotor.Direction.REVERSE);
// 
//         // Wait for the game to start (driver presses START)
//         waitForStart();
//         runtime.reset();
// 
//         // run until the end of the match (driver presses STOP)
//         while (opModeIsActive()) {
// 
//             // Setup a variable for each drive wheel to save power level for telemetry
//             double m0Power;
//             double m1Power;
//             double m2Power;
//             double m3Power;
//             // Choose to drive using either Tank Mode, or POV Mode
//             // Comment out the method that's not used.  The default below is POV.
// 
//             // POV Mode uses left stick to go forward, and right stick to turn.
//             // - This uses basic math to combine motions and is easier to drive straight.
//             double forward = -gamepad1.left_stick_y;
//             double right  =  gamepad1.left_stick_x;
// 
//             // Compensate for field orientation
//             double orientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//             double inputAngle = Math.atan2(forward, right);
//             double power = Math.sqrt(forward * forward + right * right);
//             inputAngle -= (orientation - startOrientation);
//             if (inputAngle > 2 * Math.PI) inputAngle -= 2 * Math.PI;
//             if (inputAngle < 0) inputAngle += 2 * Math.PI;
//             forward = Math.sin(inputAngle) * power;
//             right = Math.cos(inputAngle) * power;
// 
//             m0Power = forward + right;
//             m1Power = -forward + right;
//             m2Power = -forward - right;
//             m3Power = forward - right;
// 
//             double maxPower = Math.max(Math.max(Math.abs(m0Power), Math.abs(m1Power)),Math.max( Math.abs(m2Power),Math.abs (m3Power)));
//             if(maxPower>1){m0Power /=maxPower;
//                 m1Power /=maxPower;
//                 m2Power /=maxPower;
//                 m3Power /=maxPower;
// 
//             }
//             // Tank Mode uses one stick to control each wheel.
//             // - This requires no math, but it is hard to drive forward slowly and keep straight.
//             // leftPower  = -gamepad1.left_stick_y ;
//             // rightPower = -gamepad1.right_stick_y ;
// 
//             // Send calculated power to wheels
//             double motorSpeed = gamepad1.a ? 1 : 0.5;
// 
//             m0.setPower(m0Power * motorSpeed);
//             m1.setPower(m1Power * motorSpeed);
//             m2.setPower(m2Power * motorSpeed);
//             m3.setPower(m3Power * motorSpeed);
// 
//             // Show the elapsed game time and wheel power.
//             telemetry.addData("Status", "Run Time: " + runtime.toString());
//             telemetry.update();
//         }
//     }
// }
// 