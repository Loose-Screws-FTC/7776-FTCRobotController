package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="TeleOp1", group="Iterative Opmode")
public class TeleOp1 extends OpMode {
    public static int CameraDegreesXOffset = -2;

    double DistanceBasedRPM = 1600;
    public double OuttakeRPMMult = 1;

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

        this.Robot = new RobotAbstractor(hardwareMap, false);
        this.Robot.ToStartPositions();

        IMU Imu = hardwareMap.get(IMU.class, "imu");

        this.DriveSys = new Drive();
        this.DriveSys.Init(FlMotor, FrMotor, BlMotor, BrMotor, Imu, Robot.LeftDistanceSensor, Robot.RightDistanceSensor);
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

        if (gamepad2.a) {
            this.Robot.OutTakeSys.SetIsFiring(true);
        }

        if (gamepad1.aWasPressed()) {
            this.Robot.OutTakeSys.SetIsFiring(true);
        } else if (gamepad1.aWasReleased()) {
            this.Robot.OutTakeSys.SetIsFiring(false);
        }

        if (gamepad1.guideWasPressed()) {
            this.Robot.AddAction(new SequentialAction(
                new InstantAction(() -> Robot.OutTakeSys.SetIsFiring(true)),
                this.Robot.ShootAllBallsAction(),
                new InstantAction(() -> Robot.OutTakeSys.SetIsFiring(false))
            ));
        }

        if (gamepad1.bWasPressed()) {
            this.Robot.StartOutTakeBall();
        } else if (gamepad1.bWasReleased()) {
            this.Robot.OutTakeSys.ServosDown();
        }

        if (gamepad2.dpadUpWasPressed()) {
            this.Robot.AddToClassifier();
        }
        if (gamepad2.dpadDownWasPressed()) {
            this.Robot.RemoveFromClassifier();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            this.Robot.ClearClassifier();
        }

        Robot.ShowClassifierBallCountInTelemetry();

        this.OuttakeRPMMult -= 0.02 * gamepad2.left_stick_y * DeltaTime;

        telemetry.addData("raw DistanceBasedRPM", this.DistanceBasedRPM);
        telemetry.addData("mult DistanceBasedRPM", this.OuttakeRPMMult * this.DistanceBasedRPM);
        telemetry.addData("RPM mult", this.OuttakeRPMMult);

        Robot.ShouldIntake = gamepad1.right_bumper;

        Robot.IntakeShouldOuttake = gamepad2.x;

        Robot.AutoIntakeUpdate(DeltaTime);

        if (gamepad2.b) {
            this.DriveSys.ResetIMU();
        }

        if (gamepad1.left_bumper) {
            this.DriveSys.DriveSpeed = 0.2;
            this.DriveSys.TurnSpeed = 0.2;
        } else {
            this.DriveSys.DriveSpeed = 1;
            this.DriveSys.TurnSpeed = 1;
        }

        this.Robot.DecoderWheelSys.ManualAdjustAngle(
            DeltaTime * gamepad2.right_stick_x * 45
        );

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

        // Z-targeting: works like in Zelda (:
        LLResult llResult = this.Robot.Limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            for (LLResultTypes.FiducialResult fidResult : llResult.getFiducialResults()) {
                if (fidResult.getFiducialId() != 20 && fidResult.getFiducialId() != 24) {
                    continue;
                }

                // Target left/right distance from center of fov (degrees)
                double tx = fidResult.getTargetXDegrees() + CameraDegreesXOffset;
                // Target up/down distance from center of fov (degrees)
                double ty = fidResult.getTargetYDegrees();

                if (gamepad1.y) {
                    DriveSys.SetRelativeAngleTarget(Math.toRadians(tx));
                }

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);

                // Finds the distance between the camera and the currently targeted apriltag. Method is explained in detail at https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance

                double limelightMountAngle = 21.0; // Camera is 21 degrees back from vertical

                double limelightLensHeight = 12.6; // Lens height from the floor in inches

                double targetHeight = 28.0; // 28 is correct now :3

                double angleToGoal =  Math.toRadians(limelightMountAngle + ty);

                double targetDistance = (targetHeight - limelightLensHeight) / Math.tan(angleToGoal);

                telemetry.addData("Target Distance", targetDistance);

                DistanceBasedRPM = OutTake.GetRPMAt(targetDistance);
            }
        }

        this.Robot.OutTakeSys.SetVelocity((this.OuttakeRPMMult * DistanceBasedRPM) / 6000.0);

        this.DriveSys.ShouldAlignToBall = gamepad1.right_bumper;

        // emergency stop
        // if (gamepad2.left_bumper) {
        //     DriveSys.HaltTimer = 1;
        // }

        this.DriveSys.Update(DeltaTime, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        this.Robot.Update(DeltaTime);

         TelemetryPacket packet = new TelemetryPacket();

         NormalizedRGBA colors = Robot.ColorSensor.getNormalizedColors();
         packet.put("red", colors.red);
         packet.put("green", colors.green);
         packet.put("blue", colors.blue);
         packet.put("distance", ((DistanceSensor)Robot.ColorSensor).getDistance(DistanceUnit.CM));

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
