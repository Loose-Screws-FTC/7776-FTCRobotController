package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import static java.lang.System.currentTimeMillis;

@TeleOp(name="TeleOpTest0", group="Iterative Opmode")
public class TeleOpTest0 extends OpMode {
    private Drive DriveController;
    private OutTake OutTakeController;
    private Intake IntakeController;
    private DecoderWheel DecoderWheelController;
    //private Blinkin BlinkinController;

    // private double ShakePos = 0;
    private boolean LastDpadRight = false;
    private boolean LastDpadLeft = false;
    private boolean LastRightBump = false;

    private double TargetRPM = 1700;

    private Servo TiltServo;
    private double TiltServoPos = 0.51;

    Limelight3A limelight;

    private NormalizedColorSensor ColorSensor;



    @Override
    // Has to be lowercase init()
    public void init() {
        Drive.telemetry = telemetry;
        OutTake.telemetry = telemetry;
        DecoderWheel.telemetry = telemetry;
        Intake.telemetry = telemetry;

        DcMotor FlMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor FrMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor BlMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor BrMotor = hardwareMap.get(DcMotor.class, "br");

        IMU Imu = hardwareMap.get(IMU.class, "imu");

        // init DiveController for controller drive
        this.DriveController = new Drive();
        this.DriveController.Init(FlMotor, FrMotor, BlMotor, BrMotor, gamepad1, gamepad2, Imu);
        this.DriveController.SetDriveMode(Drive.DriveMode.CONTROLLER_DRIVEN);

        DcMotorEx OutLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "outr");

        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");

        this.OutTakeController = new OutTake();
        this.OutTakeController.Init(OutLeft, OutRight, OutLeftServo, OutRightServo);
        //this.OutTakeController.SetPower(0.25);

        Servo InLeftServo = hardwareMap.get(Servo.class, "intakelefts");
        Servo InRightServo = hardwareMap.get(Servo.class, "intakerights");

        DcMotor InMotor = hardwareMap.get(DcMotor.class, "intake");

        this.IntakeController = new Intake();
        this.IntakeController.Init(InLeftServo, InRightServo, InMotor);

        DcMotor DecoderWheelMotor = hardwareMap.get(DcMotor.class, "ringdrive");

        this.DecoderWheelController = new DecoderWheel();
        this.DecoderWheelController.Init(DecoderWheelMotor);

        this.DecoderWheelController.SetIntake(IntakeController);

        this.TiltServo = hardwareMap.get(Servo.class, "tiltservo");
        TiltServo.setPosition(TiltServoPos);

        ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");


        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(100);
        this.limelight.start();


        //this.BlinkinController = new Blinkin();
        //this.BlinkinController.SetColor(0.61);
    }

    // Has to be lowercase loop()
    double LastRecTime = currentTimeMillis();
    public void loop() {
        long CurrTime = currentTimeMillis();
        double DeltaTime = (CurrTime / 1000.0) - LastRecTime;
        LastRecTime = CurrTime / 1000.0;

        this.DriveController.Update(DeltaTime);
        this.OutTakeController.Update(DeltaTime);
        this.IntakeController.Update(DeltaTime);

        telemetry.addData("outtake power set", gamepad1.right_trigger);
        if (gamepad1.a) {
            this.OutTakeController.SetVelocity(gamepad1.right_trigger);
        }

        if (gamepad1.b) {
            this.OutTakeController.ServosUp();
        } else {
            this.OutTakeController.ServosDown();
        }

        if (gamepad1.right_bumper) {
            this.IntakeController.SetPower(1);
            this.IntakeController.ServosToIntake();
            this.DecoderWheelController.IntakeModeOn();
            // } else if (gamepad1.y) {
            //     this.IntakeController.SetPower(-1);
            //     this.IntakeController.DisableServos();
        } else {
            this.IntakeController.SetPower(0);
            this.IntakeController.ServosToNeutral();
            this.DecoderWheelController.IntakeModeOff();
        }

        this.DecoderWheelController.Update(DeltaTime);

        // if (gamepad1.right_bumper) {
        //     this.ShakePos += 0.5;

        //     this.DriveController.SetDriveMode(Drive.DriveMode.MANUAL);

        //     if (Math.sin(this.ShakePos) > 0) {
        //         Vector2 MoveDir = new Vector2(0, 0);
        //         this.DriveController.MoveInLocalDirectionAndTurn(MoveDir.GetNormal(), MoveDir.GetMagnitude(), 1, 1);
        //     } else {
        //         Vector2 MoveDir = new Vector2(0, 0);
        //         this.DriveController.MoveInLocalDirectionAndTurn(MoveDir.GetNormal(), MoveDir.GetMagnitude(), -1, 1);
        //     }
        // } else {
        //     this.DriveController.SetDriveMode(Drive.DriveMode.CONTROLLER_DRIVEN);
        // }

        if (gamepad1.left_bumper) {
            this.DriveController.SetDriveSpeed(0.2);
            this.DriveController.SetRotationSpeed(0.2);
        } else {
            this.DriveController.SetDriveSpeed(1);
            this.DriveController.SetRotationSpeed(1);
        }

        if (gamepad1.dpad_right && !LastDpadRight) {
            this.DecoderWheelController.RevolveRight();
        }

        if (gamepad1.dpad_left && !LastDpadLeft) {
            this.DecoderWheelController.RevolveLeft();
        }

        if (gamepad1.left_trigger > 0.5) {
            this.TargetRPM -= 350 * DeltaTime;
        }

        if (gamepad1.right_trigger > 0.5) {
            this.TargetRPM += 350 * DeltaTime;
        }

        // if (gamepad1.right_bumper && !LastRightBump) {
        //     this.DecoderWheelController.OpenToIntake();
        // }

        // if (!gamepad1.right_bumper && LastRightBump) {
        //     this.DecoderWheelController.CloseToIntake();

//        if (gamepad1.dpad_right) {
//            this.DecoderWheelController.SetPower(0.5);
//        }
//
//        else if (gamepad1.dpad_left) {
//            this.DecoderWheelController.SetPower(-0.5);
//        }

        // } else {
        //     this.DecoderWheelController.SetPower(0);
        // }
        // this.DecoderWheelController.SetPower(0);

        if (gamepad1.dpad_up) {
            this.TiltServoPos = 0.572;
            this.TiltServo.setPosition(this.TiltServoPos);
            this.TargetRPM = 1500;
        }

        if (gamepad1.dpad_down) {
            this.TiltServoPos = 0.51;
            this.TiltServo.setPosition(this.TiltServoPos);
            this.TargetRPM = 1500;
        }

        //Ball detection with color sensor
        NormalizedRGBA colors = ColorSensor.getNormalizedColors();
        if (colors.red > 0.02) {
            //Do something
        }
        //Z-targeting: works like in Zelda (:
        //Hold RightTrigger to hold orientation on Apriltag
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Target left/right distance from center of fov (degrees)
            double ty = result.getTy(); // Target up/down distance from center of fov (degrees)
            double ta = result.getTa(); // Target area (0-100% of fov)

            DriveController.setLimelightTx(result.getTx()); // Pass Tx to drive controller

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            DriveController.setLimelightTx(0);
        }

//        if (gamepad1.a && result != null && result.isValid()) {
//
//            // Finds the distance between the camera and the currently targeted apriltag. Method is explained in detail at https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
//            double targetOffsetAngle_Vertical = result.getTy();
//
//            double limelightMountAngle = 21.0; // Camera is 21 degrees back from vertical
//
//            double limelightLensHeight = 12.6; // Lens height from the floor in inches
//
//            double targetHeight = 33.0; // Height of target. 33.O IS THE CORRECT VALUE FOR THE WOODEN DOWEL MODEL!!! CHANGE BEFORE COMPETITION!!!
//
//            double angleToGoal =  Math.toRadians(limelightMountAngle + targetOffsetAngle_Vertical);
//
//            double targetDistance = (targetHeight - limelightLensHeight)/Math.tan(angleToGoal);
//
//            telemetry.addData("Target Distance",targetDistance);
//
//            this.OutTakeController.SetVelocity(((targetDistance*6)+1800)*24/60);
//        } else {
//            this.OutTakeController.Stop();
//        }



        this.DriveController.SetTargetingAprilTag(gamepad1.y);

        telemetry.addData("Red", colors.red);
        telemetry.addData("Green", colors.green);
        telemetry.addData("Blue", colors.blue);

        telemetry.addData("TiltServoPos", this.TiltServoPos);
        telemetry.addData("TargetRPM", Math.round(this.TargetRPM));
        telemetry.update();

        LastDpadRight = gamepad1.dpad_right;
        LastDpadLeft = gamepad1.dpad_left;
        LastRightBump = gamepad1.right_bumper;
    }

    public void stop() {
        this.DriveController.Stop();
        this.OutTakeController.Stop();
        this.IntakeController.Stop();
    }
}
