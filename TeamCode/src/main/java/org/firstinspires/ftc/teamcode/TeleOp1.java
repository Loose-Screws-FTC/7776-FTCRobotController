package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.limelightvision.LLResult;

import static java.lang.System.currentTimeMillis;

@Config
@TeleOp(name="TeleOp1", group="Iterative Opmode")
public class TeleOp1 extends OpMode {
    public static final double RETRACT_INTAKE_TIME = 0;
    public static final double REVOLVE_TIME = 0.1;
    public static final double REVOLVE_FINISH_TIME = 0.3;

    public static double OuttakeRPM = 1500;
    
    public RobotAbstractor Robot;
    public Drive DriveSys;

    private boolean LastDpadRight = false;
    private boolean LastDpadLeft = false;
    private boolean LastRightBump = false;

    private boolean BallDetected = false;
    private boolean RotatedAfterIntaking = false;

    private ElapsedTime runtime = new ElapsedTime();
    private double ballDetectTime = Double.POSITIVE_INFINITY;

    @Override
    // Has to be lowercase init()
    public void init() {
        Globals.telemetry = telemetry;

        runtime.reset(); // TEMP, maybe remove

        DcMotor FlMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor FrMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor BlMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor BrMotor = hardwareMap.get(DcMotor.class, "br");

        IMU Imu = hardwareMap.get(IMU.class, "imu");

        this.DriveSys = new Drive();
        this.DriveSys.Init(FlMotor, FrMotor, BlMotor, BrMotor, gamepad1, gamepad2, Imu);

        this.Robot = new RobotAbstractor(hardwareMap);
        this.DriveSys.SetDriveMode(Drive.DriveMode.CONTROLLER_DRIVEN);
    }

    // Has to be lowercase loop()
    double LastRecTime = currentTimeMillis() / 1000.0;
    public void loop() {
        double CurrTime = currentTimeMillis() / 1000.0;
        double DeltaTime = CurrTime - LastRecTime;
        LastRecTime = CurrTime;

        this.Robot.Update(DeltaTime);
        this.DriveSys.Update(DeltaTime);

        // telemetry.addData("outtake power set", gamepad1.right_trigger);
        if (gamepad1.a) {
            this.Robot.OutTakeSys.SetVelocity(OuttakeRPM / 6000.0);
        } else {
            this.Robot.OutTakeSys.SetVelocity(0);
        }

        if (gamepad1.b) {
            this.Robot.DecoderWheelSys.ClearOuttakeSlot();
            this.Robot.OutTakeSys.ServosUp();
        } else {
            this.Robot.OutTakeSys.ServosDown();
        }

        boolean currentlyIntaking = gamepad1.right_bumper;

        if (runtime.seconds() > ballDetectTime + REVOLVE_FINISH_TIME) {
            this.ballDetectTime = Double.POSITIVE_INFINITY;
        } else if (runtime.seconds() > ballDetectTime + REVOLVE_TIME) {
            if (!RotatedAfterIntaking) {
                RotatedAfterIntaking = true;
                this.Robot.DecoderWheelSys.RevolveRight();
            }
            this.Robot.IntakeSys.ServosToNeutral();
        } else if (runtime.seconds() > ballDetectTime + RETRACT_INTAKE_TIME) {
            RotatedAfterIntaking = false;
            this.Robot.IntakeSys.ServosToNeutral();
        } else if (currentlyIntaking) {
            this.Robot.IntakeSys.ServosToIntake();
        } else {
            this.Robot.IntakeSys.ServosToNeutral();
        }

        if (currentlyIntaking) {
            this.Robot.DecoderWheelSys.IntakeModeOn();
        } else {
            this.Robot.DecoderWheelSys.IntakeModeOff();
        }

        this.Robot.IntakeSys.SetPower(currentlyIntaking || !this.Robot.DecoderWheelSys.GetIsAtTarget()
            ? 1.0 : 0.0);

        // if (gamepad1.right_bumper) {
        //     this.ShakePos += 0.5;

        //     this.Robot.DriveSys.SetDriveMode(Drive.DriveMode.MANUAL);

        //     if (Math.sin(this.ShakePos) > 0) {
        //         Vector2 MoveDir = new Vector2(0, 0);
        //         this.Robot.DriveSys.MoveInLocalDirectionAndTurn(MoveDir.GetNormal(), MoveDir.GetMagnitude(), 1, 1);
        //     } else {
        //         Vector2 MoveDir = new Vector2(0, 0);
        //         this.Robot.DriveSys.MoveInLocalDirectionAndTurn(MoveDir.GetNormal(), MoveDir.GetMagnitude(), -1, 1);
        //     }
        // } else {
        //     this.Robot.DriveSys.SetDriveMode(Drive.DriveMode.CONTROLLER_DRIVEN);
        // }

        if (gamepad1.left_bumper) {
            this.DriveSys.SetDriveSpeed(0.2);
            this.DriveSys.SetRotationSpeed(0.2);
        } else {
            this.DriveSys.SetDriveSpeed(1);
            this.DriveSys.SetRotationSpeed(1);
        }

        if (gamepad1.dpad_right && !LastDpadRight) {
            this.Robot.DecoderWheelSys.RevolveRight();
        }

        if (gamepad1.dpad_left && !LastDpadLeft) {
            this.Robot.DecoderWheelSys.RevolveLeft();
        }

        if (gamepad1.dpad_up) {
            this.Robot.DecoderWheelSys.RevolveToColor(DecoderWheel.BallColor.PURPLE);
        }

        if (gamepad1.dpad_down) {
            this.Robot.DecoderWheelSys.RevolveToColor(DecoderWheel.BallColor.GREEN);
        }

//        if (gamepad1.left_trigger > 0.5) {
//            this.TargetRPM -= 350 * DeltaTime;
//        }

//        if (gamepad1.right_trigger > 0.5) {
//            this.TargetRPM += 350 * DeltaTime;
//        }

        // if (gamepad1.right_bumper && !LastRightBump) {
        //     this.Robot.DecoderWheelSys.OpenToIntake();
        // }

        // if (!gamepad1.right_bumper && LastRightBump) {
        //     this.Robot.DecoderWheelSys.CloseToIntake();

//        if (gamepad1.dpad_right) {
//            this.Robot.DecoderWheelSys.SetPower(0.5);
//        }
//
//        else if (gamepad1.dpad_left) {
//            this.Robot.DecoderWheelSys.SetPower(-0.5);
//        }

        // } else {
        //     this.Robot.DecoderWheelSys.SetPower(0);
        // }
        // this.Robot.DecoderWheelSys.SetPower(0);

//        if (gamepad1.dpad_up) {
//            this.TiltServoPos = 0.572;
//            this.TiltServo.setPosition(this.TiltServoPos);
//            this.TargetRPM = 1500;
//        }
//
//        if (gamepad1.dpad_down) {
//            this.TiltServoPos = 0.51;
//            this.TiltServo.setPosition(this.TiltServoPos);
//            this.TargetRPM = 1500;
//        }

        // Ball detection with color sensor
        // "capturetime" is used to allow the ball to settle before rotating and to prevent misfires (caused by sensor looking through a hole in the ball or a momentary bad value)
        // All three color values being compared to 0.01 will cause the system to trigger on essentially any object with color, should maybe be tuned for specific ball colors (or indexing could be handled elsewhere)
        NormalizedRGBA colors = this.Robot.ColorSensor.getNormalizedColors();
        if (Double.isInfinite(ballDetectTime)) {
            if (colors.red > 0.01 || colors.green > 0.01 || colors.blue > 0.01) { // Color sensor values typically float between 0.001 and 0.002 when looking at nothing, and are normally between 0.01 and 0.03 for colored objects (depending on the color)
                this.Robot.DecoderWheelSys.SetIntakedColor(DecoderWheel.DetermineBallColor(colors));
                this.BallDetected = true;
                this.ballDetectTime = runtime.seconds();
            } else {
                this.BallDetected = false;
            }
        }

        //Z-targeting: works like in Zelda (:
        //Hold RightTrigger to hold orientation on Apriltag
        LLResult result = this.Robot.Limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Target left/right distance from center of fov (degrees)
            double ty = result.getTy(); // Target up/down distance from center of fov (degrees)
            double ta = result.getTa(); // Target area (0-100% of fov)

            this.DriveSys.setLimelightTx(result.getTx()); // Pass Tx to drive system

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            this.DriveSys.setLimelightTx(0);
        }

        if (result != null && result.isValid()) {

            // Finds the distance between the camera and the currently targeted apriltag. Method is explained in detail at https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
            double targetOffsetAngle_Vertical = result.getTy();

            double limelightMountAngle = 21.0; // Camera is 21 degrees back from vertical

            double limelightLensHeight = 12.6; // Lens height from the floor in inches

            double targetHeight = 33.0; // Height of target. 33.O IS THE CORRECT VALUE FOR THE WOODEN DOWEL MODEL!!! CHANGE BEFORE COMPETITION!!!

            double angleToGoal =  Math.toRadians(limelightMountAngle + targetOffsetAngle_Vertical);

            double targetDistance = (targetHeight - limelightLensHeight)/Math.tan(angleToGoal);

            telemetry.addData("Target Distance",targetDistance);
        }

        this.DriveSys.SetTargetingAprilTag(gamepad1.y);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("red", colors.red);
        packet.put("green", colors.green);
        packet.put("blue", colors.blue);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("Ball Detected: ", BallDetected);

//        telemetry.addData("TargetRPM", Math.round(this.TargetRPM));

        telemetry.update();

        LastDpadRight = gamepad1.dpad_right;
        LastDpadLeft = gamepad1.dpad_left;
        LastRightBump = gamepad1.right_bumper;
    }

    public void stop() {
        this.DriveSys.Stop();
        this.Robot.Stop();
    }
}
