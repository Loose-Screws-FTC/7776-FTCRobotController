package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Drive {
    private DcMotor FlMotor;
    private DcMotor FrMotor;
    private DcMotor BlMotor;
    private DcMotor BrMotor;

    private IMU Imu;

    private DistanceSensor LeftDistanceSensor;
    private DistanceSensor RightDistanceSensor;

    private double Heading = 0;
    private double TargetRot = Double.NaN;

    public double DriveSpeed = 1;
    public double TurnSpeed = 1;

    public static double StrafeSpeed = 0.3;

    public static double DefaultMaintainRotP = 1.5;
    public static double SpecificMaintainRotP = 3;
    public double MaintainRotP = DefaultMaintainRotP;
    public static double RotStoreSpeed = 0.05;

    public boolean ShouldAlignToBall = false;

    public double HaltTimer = 0;

    public void Init(DcMotor FlDrive, DcMotor FrDrive, DcMotor BlDrive, DcMotor BrDrive, IMU Imu, DistanceSensor LeftDistanceSensor, DistanceSensor RightDistanceSensor) {
        this.FlMotor = FlDrive;
        this.FrMotor = FrDrive;
        this.BlMotor = BlDrive;
        this.BrMotor = BrDrive;

        this.FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.FlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.FrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.FlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.FrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.BrMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU.Parameters IMUParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS,
                0,
                0,
                (float)Math.PI / 4,
                0
            ))
        );

        this.Imu = Imu;
        this.Imu.initialize(IMUParameters);
        this.Imu.resetYaw();

        this.LeftDistanceSensor = LeftDistanceSensor;
        this.RightDistanceSensor = RightDistanceSensor;
    }

    public void ResetIMU() {
        TargetRot = Double.NaN;
        this.Imu.resetYaw();
    }

    public void Update(double deltaTime, double inputX, double inputY, double inputTurn) {
        if (HaltTimer > 0) {
            HaltTimer -= deltaTime;
            Stop();
            return;
        }

        Heading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (Double.isNaN(TargetRot)) {
            double yRotVel = this.Imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
            if (Math.abs(yRotVel) < RotStoreSpeed) {
                TargetRot = Heading;
                MaintainRotP = DefaultMaintainRotP;
            }
        }

        Vector2 forward = Vector2.AtAngle(Heading);

        Vector2 inputLateral = new Vector2(inputX, inputY);
        inputLateral.Pow(3);

        Vector2 lateral;
        if (ShouldAlignToBall) {
            double leftDistance = LeftDistanceSensor.getDistance(DistanceUnit.CM);
            double rightDistance = RightDistanceSensor.getDistance(DistanceUnit.CM);
            boolean ballOnLeft = !Double.isNaN(leftDistance);
            boolean ballOnRight = !Double.isNaN(rightDistance);

            double leftPower = 0;
            if (ballOnLeft && !ballOnRight) {
                leftPower = StrafeSpeed;
            } else if (ballOnRight && !ballOnLeft) {
                leftPower = -StrafeSpeed;
            }

            lateral = new Vector2(leftPower, inputLateral.GetMagnitude() * DriveSpeed);
        } else {
            lateral = inputLateral;
            lateral.Scale(-DriveSpeed);
            lateral.ComplexMultiply(forward);
        }

        double turn = 0;
        if (Math.abs(inputTurn) > 0.01) {
            turn = Math.pow(inputTurn, 3) * TurnSpeed;
            TargetRot = Double.NaN;
        } else if (!Double.isNaN(TargetRot)) {
            double turnGoal = AngleTo2PI(Heading - TargetRot + Math.PI) - Math.PI;
            turn = turnGoal * MaintainRotP;
        }

        double FlPower = lateral.Y - lateral.X + turn;
        double FrPower = lateral.Y + lateral.X - turn;
        double BlPower = lateral.Y + lateral.X + turn;
        double BrPower = lateral.Y - lateral.X - turn;

        this.FlMotor.setPower(FlPower);
        this.FrMotor.setPower(FrPower);
        this.BlMotor.setPower(BlPower);
        this.BrMotor.setPower(BrPower);
    }

    double AngleTo2PI(double a) {
        a %= Math.PI * 2;
        if (a < 0) a += Math.PI * 2;
        return a;
    }

    // remember, angle must be in radians
    public void SetRelativeAngleTarget(double angle) {
        TargetRot = Heading - angle;
        MaintainRotP = SpecificMaintainRotP;
    }

    public void Stop() {
        this.FlMotor.setPower(0);
        this.FrMotor.setPower(0);
        this.BlMotor.setPower(0);
        this.BrMotor.setPower(0);
    }
}
