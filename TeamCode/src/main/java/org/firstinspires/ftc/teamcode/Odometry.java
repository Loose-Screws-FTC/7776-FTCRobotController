package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Odometry {
    private DcMotor XOdom;
    private DcMotor YOdom;
    private IMU Imu;

    private double XPos = 0;
    private double YPos = 0;
    private double Rot = 0; // Rotation

    private int LastXOdomTicks = 0;
    private int LastYOdomTicks = 0;
    private double LastRot = 0;

    private double WheelRadius = 1; // in inches
    private double TicksPerRevolution = 8192; // example value, change as needed
    private double OdometerCircumference = 2 * Math.PI * WheelRadius;

    private double XOdomOffset = 0; // in inches
    private double YOdomOffset = 0; // in inches

    public void Init(DcMotor XOdometry, DcMotor YOdometry, IMU Imu2) {
        this.XOdom = XOdometry;
        this.YOdom = YOdometry;

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

        this.Imu = Imu2;
        this.Imu.initialize(IMUParameters);

        this.Imu.resetYaw();

        this.XOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.YOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.XOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.YOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ResetIMU() {
        this.Imu.resetYaw();
    }

    public void Update(double DeltaTime) {
        int CurrXOdomTicks = this.XOdom.getCurrentPosition();
        int CurrYOdomTicks = this.YOdom.getCurrentPosition();

        int DeltaXTicks = CurrXOdomTicks - this.LastXOdomTicks;
        int DeltaYTicks = CurrYOdomTicks - this.LastYOdomTicks;

        this.LastXOdomTicks = CurrXOdomTicks;
        this.LastYOdomTicks = CurrYOdomTicks;

        this.Rot = this.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double DeltaRot = this.Rot - this.LastRot;
        this.LastRot = this.Rot;

        double CosDeltaRot = Math.cos(Math.toRadians(DeltaRot));
        double SinDeltaRot = Math.sin(Math.toRadians(DeltaRot));

        double DeltaXInches = (DeltaXTicks / TicksPerRevolution) * OdometerCircumference;
        double DeltaYInches = (DeltaYTicks / TicksPerRevolution) * OdometerCircumference;

        double GlobalDeltaX = DeltaXInches * CosDeltaRot - DeltaYInches * SinDeltaRot;
        double GlobalDeltaY = DeltaXInches * SinDeltaRot + DeltaYInches * CosDeltaRot;

        this.XPos += GlobalDeltaX;
        this.YPos += GlobalDeltaY;
    }

    public double GetX() {
        return this.XPos + XOdomOffset;
    }

    public double GetY() {
        return this.YPos + YOdomOffset;
    }

    /**
     * @return Heading in radians
     */
    public double GetHeading() {
        return this.Rot;
    }
}
