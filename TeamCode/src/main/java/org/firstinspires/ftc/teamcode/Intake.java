package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo LServ;
    private Servo RServ;

    private DcMotor Motor;

    private double Speed = 0;
    private double TargetSpeed = 0;
    private double RampUpSpeed = 4;

    public void Init(Servo LeftServo, Servo RightServo, DcMotor IntakeMotor) {
        this.LServ = LeftServo;
        this.RServ = RightServo;

        this.Motor = IntakeMotor;

        this.Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.ServosToNeutral();
    }

    public void SetServosToPos(double pos) {
        this.LServ.setPosition(0.46 - pos);
        this.RServ.setPosition(0.46 + pos);
    }

    public void Update(double DeltaTime) {
        this.Speed += (this.TargetSpeed - this.Speed) * DeltaTime * this.RampUpSpeed;
        this.Motor.setPower(this.Speed);
    }

    public void SetPower(double Power) {
        this.TargetSpeed = Power;
    }

    public void EnableServos() {
        ((ServoImplEx) this.LServ).setPwmEnable();
        ((ServoImplEx) this.RServ).setPwmEnable();
    }

    public void ServosToNeutral() {
        this.EnableServos();
        this.SetServosToPos(0.01);
    }

    public void ServosToIntake() {
        this.EnableServos();
        this.SetServosToPos(0.085);
    }

    public void DisableServos() {
        ((ServoImplEx) this.LServ).setPwmDisable();
        ((ServoImplEx) this.RServ).setPwmDisable();
    }

    public void Stop() {
        this.Motor.setPower(0);
        Speed = 0;
        TargetSpeed = 0;
    }
}
