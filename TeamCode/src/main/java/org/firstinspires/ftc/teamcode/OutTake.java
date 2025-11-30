package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutTake {
    public static Telemetry telemetry;

    private DcMotor LMotor;
    private DcMotor RMotor;

    private Servo LServ;
    private Servo RServ;

    private double LeftTicksPerRev = 28;
    private double RightTicksPerRev = 28;

    private double LastLRevs = 0;
    private double LastRRevs = 0;

    private double AproxMaxRPM = 6000;

    private double PowerChangeSpeed = 1;

    private double LPower = 1;
    private double RPower = 1;
    private double WantedRPM = 5100;
    private boolean RunningToRPM = false;

    private boolean ReachedTargetRPM = false;
    private double ReachingTargetRPM = 0; // RPM

    public void Init(DcMotor LeftMotor, DcMotor RightMotor, Servo LeftServo, Servo RightServo) {
        LMotor = LeftMotor;
        RMotor = RightMotor;

        LServ = LeftServo;
        RServ = RightServo;

        LServ.setDirection(Servo.Direction.REVERSE);
        RServ.setDirection(Servo.Direction.FORWARD);

        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void Update(double DeltaTime) {
        double LPos = LMotor.getCurrentPosition();
        double RPos = RMotor.getCurrentPosition();

        double LRevs = LPos / LeftTicksPerRev;
        double RRevs = RPos / RightTicksPerRev;

        double LeftRPM = (LRevs - LastLRevs) / (DeltaTime / 60);
        double RightRPM = (RRevs - LastRRevs) / (DeltaTime / 60);

        if (this.RunningToRPM) {
            if (LeftRPM > this.WantedRPM || RightRPM > this.WantedRPM) {
                this.ReachedTargetRPM = true;
            }

            LMotor.setPower(this.LPower);
            RMotor.setPower(this.RPower);

            if (this.ReachedTargetRPM) {
                LMotor.setPower(this.WantedRPM / this.AproxMaxRPM);
                RMotor.setPower(this.WantedRPM / this.AproxMaxRPM);
            } else {
                LMotor.setPower(1);
                RMotor.setPower(1);
            }
        }

        String LeftMsg = String.valueOf(Math.round(LeftRPM)) + " / " + String.valueOf(this.AproxMaxRPM) + " (" + String.valueOf(Math.round(LeftRPM / this.AproxMaxRPM)) + "%)";
        String RightMsg = String.valueOf(Math.round(RightRPM)) + " / " + String.valueOf(this.AproxMaxRPM) + " (" + String.valueOf(Math.round(RightRPM / this.AproxMaxRPM)) + "%)";

        telemetry.addData("Left RPM", LeftMsg);
        telemetry.addData("Right RPM", RightMsg);
        telemetry.addData("LPower", this.LPower);
        telemetry.addData("RPower", this.RPower);

        LastLRevs = LRevs;
        LastRRevs = RRevs;
    }

    public void SetPower(double Power) {
        LMotor.setPower(Power);
        RMotor.setPower(Power);
        this.RunningToRPM = false;
    }

    public void RunToRPM(double RPM) {
        this.WantedRPM = RPM;
        if (!this.RunningToRPM) {
            this.LPower = RPM;
            this.RPower = RPM;
        }
        if (RPM != this.ReachingTargetRPM) {
            this.ReachedTargetRPM = false;
        }
        this.ReachingTargetRPM = RPM;
        this.RunningToRPM = true;
    }

    public void StopRunToRPM() {
        if (this.RunningToRPM) {
            LMotor.setPower(0);
            RMotor.setPower(0);
        }
        this.ReachingTargetRPM = 0;
        this.RunningToRPM = false;
    }

    public void ServosUp() {
        LServ.setPosition(0.625);
        RServ.setPosition(0.625);
    }

    public void ServosDown() {
        LServ.setPosition(0.49);
        RServ.setPosition(0.475);
    }

    public void Stop() {
        LMotor.setPower(0);
        RMotor.setPower(0);
    }
}
