package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OutTake {
    private DcMotorEx LMotor;
    private DcMotorEx RMotor;

    private Servo LServ;
    private Servo RServ;

    private double LeftTicksPerRev = 28;
    private double RightTicksPerRev = 28;

    private double ApproxMaxRPM = 6000;
    // private double WantedRPM = 5100;
    // private boolean RunningToRPM = false;

    private double MaxTicksPerSecond = 28.0 / (60.0 / ApproxMaxRPM);

    private double VelocityMargin = 0.01;

    private boolean IsFiring = false;
    private double TargetVelocity = 0;

    private boolean AreServosUp = false;

    public static double NEW_P = 180;
    public static double NEW_I = 0;
    public static double NEW_D = 100;
    public static double NEW_F = 12;

    public static double ComputedRPMMultiplier = 0.95 * 0.98;

    public void Init(DcMotorEx LeftMotor, DcMotorEx RightMotor, Servo LeftServo, Servo RightServo, Servo TiltServo) {
        LMotor = LeftMotor;
        RMotor = RightMotor;

        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LServ = LeftServo;
        RServ = RightServo;

        LServ.setDirection(Servo.Direction.REVERSE);
        RServ.setDirection(Servo.Direction.FORWARD);
    }

    public void Update(double DeltaTime) {
        double LPos = LMotor.getCurrentPosition();
        double RPos = RMotor.getCurrentPosition();

        double LRevs = LPos / LeftTicksPerRev;
        double RRevs = RPos / RightTicksPerRev;

//        double LeftTicks = (LRevs - LastLRevs);
//        double RightTicks = (RRevs - LastRRevs);
        double LeftRPM = LMotor.getVelocity() / LeftTicksPerRev * 60;
        double RightRPM = RMotor.getVelocity() / RightTicksPerRev * 60;
//        double LeftRPM = LeftTicks / (DeltaTime / 60);
//        double RightRPM = RightTicks / (DeltaTime / 60);

//        PIDCoefficients pidOrig = LMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        LMotor.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        RMotor.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);



//        if (this.RunningToRPM) {
//            if (LeftRPM > this.WantedRPM || RightRPM > this.WantedRPM) {
//                this.ReachedTargetRPM = true;
//            }
//
//            LMotor.setPower(this.LPower);
//            RMotor.setPower(this.RPower);
//
//            if (this.ReachedTargetRPM) {
//                LMotor.setPower(this.WantedRPM / this.AproxMaxRPM);
//                RMotor.setPower(this.WantedRPM / this.AproxMaxRPM);
//            } else {
//                LMotor.setPower(1);
//                RMotor.setPower(1);
//            }
//        }

        String LeftMsg = String.valueOf(Math.round(LeftRPM)) + " / " + String.valueOf(this.ApproxMaxRPM) + " (" + String.valueOf(Math.round(LeftRPM / this.ApproxMaxRPM)) + "%)";
        String RightMsg = String.valueOf(Math.round(RightRPM)) + " / " + String.valueOf(this.ApproxMaxRPM) + " (" + String.valueOf(Math.round(RightRPM / this.ApproxMaxRPM)) + "%)";

        Globals.telemetry.addData("Left RPM", LeftMsg);
        Globals.telemetry.addData("Right RPM", RightMsg);
        // Globals.telemetry.addData("LPower", this.LPower);
        // Globals.telemetry.addData("RPower", this.RPower);

        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("LeftTicks", LeftTicks);
//        packet.put("RightTicks", RightTicks);
        packet.put("LeftRPM", LeftRPM);
        packet.put("RightRPM", RightRPM);
        packet.put("ArmsUp", AreServosUp ? TargetVelocity * this.ApproxMaxRPM: 0.0);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // LastLRevs = LRevs;
        // LastRRevs = RRevs;
    }

    // https://www.desmos.com/calculator/xx5wnlycwg
    private static double GetRawRPMAt(double distance) {
        if (distance < 53.5) return 5 * distance + 1332.5;
        if (distance < 68) return 1600;
        // return (-182.52615 / (1 + Math.exp(0.181724 * distance - 19.42445))) + 1781.5753;
        return (-205.9896 / (1 + Math.exp(0.158699 * distance - 17.30768))) + 1804.51045;
    }

    public static double GetRPMAt(double distance) {
        return GetRawRPMAt(distance) * ComputedRPMMultiplier;
    }

    public void SetVelocity(double Velocity) {
        this.TargetVelocity = Velocity;
        UpdateMotorVelocities();
    }

    public void SetIsFiring(boolean isFiring) {
        IsFiring = isFiring;
        UpdateMotorVelocities();
    }

    private void UpdateMotorVelocities() {
        if (IsFiring) {
            LMotor.setVelocity(TargetVelocity * MaxTicksPerSecond);
            RMotor.setVelocity(TargetVelocity * MaxTicksPerSecond);
        } else {
            LMotor.setVelocity(0);
            RMotor.setVelocity(0);
        }
    }

    public double GetCurrentVelocity() {
        return (LMotor.getVelocity() + RMotor.getVelocity()) / MaxTicksPerSecond / 2;
    }

    public boolean IsAtVelocity() {
        if (IsFiring) {
            return Math.abs(GetCurrentVelocity() - TargetVelocity) < VelocityMargin;
        } else {
            return true;
        }
    }

//    public void RunToRPM(double RPM) {
//        this.WantedRPM = RPM;
//        if (!this.RunningToRPM) {
//            this.LPower = RPM;
//            this.RPower = RPM;
//        }
//        if (RPM != this.ReachingTargetRPM) {
//            this.ReachedTargetRPM = false;
//        }
//        this.ReachingTargetRPM = RPM;
//        this.RunningToRPM = true;
//    }

//    public void StopRunToRPM() {
//        if (this.RunningToRPM) {
//            LMotor.setPower(0);
//            RMotor.setPower(0);
//        }
//        this.ReachingTargetRPM = 0;
//        this.RunningToRPM = false;
//    }

    public void ServosUp() {
        AreServosUp = true;
        LServ.setPosition(0.65);
        RServ.setPosition(0.65);
    }

    public void ServosDown() {
        AreServosUp = false;
        LServ.setPosition(0.49);
        RServ.setPosition(0.475);
    }

    public void Stop() {
        SetVelocity(0);
    }
}
