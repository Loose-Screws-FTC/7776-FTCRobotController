package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="TeleOp1", group="Iterative Opmode")
public class TeleOp1 extends OpMode {
    public static int CameraPixelsXOffset = -2;

    double DistanceBasedRPM = 1600;
    public double OuttakeRPMMult = 1;

    //Distance measurements of the left and right intake sensors. Probably there's a better way to do this but I don't know how -Rowan
    public static double LeftDistance;
    public static double RightDistance;

    public RobotAbstractor Robot;
    public Drive DriveSys;

    @Override
    // Has to be lowercase init()
    public void init() {
        Globals.telemetry = telemetry;

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
    double LastRecTime = Double.NaN;
    public void loop() {
        double CurrTime = Robot.Runtime.seconds();
        if (Double.isNaN(LastRecTime)) {
            LastRecTime = CurrTime;
        }
        double DeltaTime = CurrTime - LastRecTime;
        LastRecTime = CurrTime;

        this.Robot.Update(DeltaTime);
        this.DriveSys.Update(DeltaTime);

        //Update the distance variables once per loop
        //Data comes from RobotAbstractor.java and is immediately passed through to Drive.java :| (this is not fantastic)
        LeftDistance = this.Robot.LeftDistanceSensor.getDistance(DistanceUnit.CM);
        RightDistance = this.Robot.RightDistanceSensor.getDistance(DistanceUnit.CM);

        if (gamepad1.a) {
            this.Robot.OutTakeSys.SetVelocity((this.OuttakeRPMMult * DistanceBasedRPM) / 6000.0);
        } else {
            this.Robot.OutTakeSys.SetVelocity(0);
        }

        if (gamepad1.guideWasPressed()) {
            this.Robot.AddAction(this.Robot.ShootAllBallsAction());
        }

        if (gamepad1.bWasPressed()) {
            this.Robot.StartOutTakeBall();
        } else if (gamepad1.bWasReleased()) {
            this.Robot.OutTakeSys.ServosDown();
        }

        if (gamepad2.dpad_up) {
            this.OuttakeRPMMult -= 0.02 * DeltaTime;
        }
        if (gamepad2.dpad_down) {
            this.OuttakeRPMMult += 0.02 * DeltaTime;
        }
        telemetry.addData("raw DistanceBasedRPM", this.DistanceBasedRPM);
        telemetry.addData("mult DistanceBasedRPM", this.OuttakeRPMMult * this.DistanceBasedRPM);
        telemetry.addData("RPM mult", this.OuttakeRPMMult);

        Robot.ShouldIntake = gamepad1.right_bumper;
        Robot.AutoIntakeUpdate(DeltaTime);

        if (gamepad2.b) {
            this.DriveSys.ResetIMU();
        }

        if (gamepad1.left_bumper) {
            this.DriveSys.SetDriveSpeed(0.2);
            this.DriveSys.SetRotationSpeed(0.2);
        } else {
            this.DriveSys.SetDriveSpeed(1);
            this.DriveSys.SetRotationSpeed(1);
        }

        if (gamepad1.dpadRightWasPressed()) {
            this.Robot.DecoderWheelSys.RevolveRight();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            this.Robot.DecoderWheelSys.RevolveLeft();
        }

        if (gamepad1.dpad_up) {
            this.Robot.DecoderWheelSys.RevolveToColor(BallColor.PURPLE);
        }

        if (gamepad1.dpad_down) {
            this.Robot.DecoderWheelSys.RevolveToColor(BallColor.GREEN);
        }

        //Z-targeting: works like in Zelda (:
        //Hold RightTrigger to hold orientation on Apriltag
        LLResult result = this.Robot.Limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Target left/right distance from center of fov (degrees)
            double tx = result.getTx() + CameraPixelsXOffset;
            // Target up/down distance from center of fov (degrees)
            double ty = result.getTy();
            // Target area (0-100% of fov)
            double ta = result.getTa();

            this.DriveSys.setLimelightTx(tx); // Pass Tx to drive system

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

            double targetHeight = 28.0; // 28 is correct now :3

            double angleToGoal =  Math.toRadians(limelightMountAngle + targetOffsetAngle_Vertical);

            double targetDistance = (targetHeight - limelightLensHeight)/Math.tan(angleToGoal);

            telemetry.addData("Target Distance",targetDistance);

            DistanceBasedRPM = OutTake.GetRPMAt(targetDistance);
        }

        this.DriveSys.SetTargetingAprilTag(gamepad1.y);

        this.DriveSys.SetTargetingBall(gamepad1.right_bumper);

        TelemetryPacket packet = new TelemetryPacket();

        NormalizedRGBA colors = Robot.ColorSensor.getNormalizedColors();
        packet.put("red", colors.red);
        packet.put("green", colors.green);
        packet.put("blue", colors.blue);

        packet.put("Left Distance", this.Robot.LeftDistanceSensor.getDistance(DistanceUnit.CM));
        packet.put("Right Distance", this.Robot.RightDistanceSensor.getDistance(DistanceUnit.CM));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        telemetry.addData("Ball Detected: ", BallDetected);

//        telemetry.addData("TargetRPM", Math.round(this.TargetRPM));

        telemetry.update();
    }

    public void stop() {
        this.DriveSys.Stop();
        this.Robot.Stop();
    }
}
