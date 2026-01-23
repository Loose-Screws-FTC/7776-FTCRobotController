package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class DecoderWheel {
    private DcMotorEx Motor;

    private int IntakeSlot = 1;

    private double AddedAngleToRevolveOneStep = 120;
    private double TicksPerRev = 28 * 6.2;
    private double TargetAngle = 0;
    private double CurrAngle = 0;

    public static double MaxMotorPower = 0.6;
    public static double CloseMotorPower = 0.3;
    public static double NoBallsPowerMultiplier = 0.5;
    public static double AcceptableAngleDeviation = 5;
    public static double CloseAngleDeviation = 15;
    public static double PositionV = 20;
    public static double VelocityP = 60;
    public static double VelocityI = 20;
    public static double VelocityD = 10;
    public static double VelocityF = 30;
    public static double MaxPower = 0.1;
    public static int PowerCutDeviation = 10;

    private boolean IsCurrentlyOpenToIntake = false;

    private boolean AtTarget = false;

    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    // Index 0 should always be the one in front of the outtake wheels.
    private List<BallColor> BallsInWheel = new ArrayList<>();

    public void Init(DcMotor WheelMotor) {
        this.Motor = (DcMotorEx)WheelMotor;

        this.Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.Motor.setTargetPosition(0);
//        this.Motor.setPower(MotorPower);
//        this.Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.Motor.setTargetPosition(0);
//        this.Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        PIDFCoefficients coefs = this.Motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addLine(coefs.toString());
//
//        this.Motor.setPositionPIDFCoefficients(30);
//
//        this.Motor.setVelocityPIDFCoefficients(15, 5, 15, 0);

        this.BallsInWheel.add(BallColor.NONE);
        this.BallsInWheel.add(BallColor.NONE);
        this.BallsInWheel.add(BallColor.NONE);
    }

    public void Update(double DeltaTime) {
        Globals.telemetry.addData("ball1", this.BallsInWheel.get(0));
        Globals.telemetry.addData("ball2", this.BallsInWheel.get(1));
        Globals.telemetry.addData("ball3", this.BallsInWheel.get(2));

//        Globals.telemetry.addData("ballorder", BallOrder.GameOrder.toString());

//        this.Motor.setPositionPIDFCoefficients(PositionV);
//        this.Motor.setVelocityPIDFCoefficients(VelocityP, VelocityI, VelocityD, VelocityF);

        this.CurrAngle = this.Motor.getCurrentPosition() / TicksPerRev * 360;
//        Globals.telemetry.addData("CurrAngle", CurrAngle);
//        Globals.telemetry.addData("TargetAngle", TargetAngle);
//        this.Motor.setTargetPosition((int)(TargetAngle / 360.0 * TicksPerRev));
//        double AngleDeviation = Math.abs(this.CurrAngle - this.TargetAngle);
//        this.AtTarget = AngleDeviation <= AcceptableAngleDeviation;

//        if (AngleDeviation <= PowerCutDeviation) {
//            this.Motor.setPower(0);
//        } else {
//            this.Motor.setPower(MaxPower);
//        }

//        PIDFCoefficients coefs = this.Motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        Globals.telemetry.addData("p", Double.toString(coefs.p));
//        Globals.telemetry.addData("i", Double.toString(coefs.i));
//        Globals.telemetry.addData("d", Double.toString(coefs.d));
//        Globals.telemetry.addData("f", Double.toString(coefs.f));

        boolean isClose = Math.abs(this.TargetAngle - this.CurrAngle) < CloseAngleDeviation;
        double baseMotorPower = isClose ? CloseMotorPower : MaxMotorPower;
        if (!HasAnyBalls()) baseMotorPower *= NoBallsPowerMultiplier;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("curr angle", this.CurrAngle);
        packet.put("target angle", this.TargetAngle);
        packet.put("AcceptableAngleDeviation", AcceptableAngleDeviation);

        if (this.CurrAngle < this.TargetAngle - AcceptableAngleDeviation) {
            packet.put("power", baseMotorPower);
            this.Motor.setPower(baseMotorPower);
            AtTarget = false;
        } else if (this.CurrAngle > this.TargetAngle + AcceptableAngleDeviation) {
            packet.put("power", -baseMotorPower);
            this.Motor.setPower(-baseMotorPower);
            AtTarget = false;
        } else {
            packet.put("power", 0);
            this.Motor.setPower(0);
            AtTarget = true;
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        //telemetry.addData("Current angle", this.CurrAngle);
        //telemetry.addData("Motor", this.Motor.getCurrentPosition());
//        this.Motor.setPositionPIDFCoefficients(RobotConfig.DecoderWheelPosP);
//        this.Motor.setVelocityPIDFCoefficients(
//            RobotConfig.DecoderWheelVelP,
//            RobotConfig.DecoderWheelVelI,
//            RobotConfig.DecoderWheelVelD,
//            0 // F
//        );
//
//        telemetry.addData("target angle", this.TargetAngle);
//        this.Motor.setTargetPosition((int)(this.TargetAngle * TicksPerRev / 360.0));
    }

    public BallColor GetBallColorAt(int pos) {
        return this.BallsInWheel.get(pos);
    }

    public boolean IsAtTarget() {
        return this.AtTarget;
    }

    /*
    public void SetAngle(double Angle) {
        this.CurrentAngle = Angle;
        int TargetTicks = (int)(CurrentAngle / 360.0 * TicksPerRev);
        this.Motor.setTargetPosition(TargetTicks);
        this.Motor.setPower(0.5);
    }
     */

    public void RevolveRight() {
        this.TargetAngle += AddedAngleToRevolveOneStep;

        Collections.rotate(BallsInWheel, 1);
    }

    public void RevolveLeft() {
        this.TargetAngle -= AddedAngleToRevolveOneStep;

        Collections.rotate(BallsInWheel, -1);
    }

    public boolean HasAnyBalls() {
        for (BallColor color : BallsInWheel) {
            if (color != BallColor.NONE) return true;
        }
        return false;
    }

    public void SetIntakedColor(BallColor Color) {
        BallsInWheel.set(IntakeSlot, Color);
    }

    public void ClearOuttakeSlot() {
        BallsInWheel.set(0, BallColor.NONE);
    }

    public void SetCurrentColors(BallColor ball1, BallColor ball2, BallColor ball3) {
        BallsInWheel.set(0, ball1);
        BallsInWheel.set(1, ball2);
        BallsInWheel.set(2, ball3);
    }

    public static BallColor DetermineBallColor(NormalizedRGBA colors) {
        if (colors.green > colors.blue) {
            return BallColor.GREEN;
        } else {
            return BallColor.PURPLE;
        }
    }

    public boolean RevolveToColor(BallColor Color) {
        if (this.IsCurrentlyOpenToIntake)
            return false;

        if (this.BallsInWheel.get(0) == Color) {
            return true;
        } else if (this.BallsInWheel.get(1) == Color) {
            this.RevolveLeft();
            return true;
        } else if (this.BallsInWheel.get(2) == Color) {
            this.RevolveRight();
            return true;
        } else {
            return false;
        }
    }

    // public void OpenToIntake() {
    //     this.IsCurrentlyOpenToIntake = true;
    //     this.TargetAngle += AddedAngleToRevolveOneStep / 2;
    // }

    // public void CloseToIntake() {
    //     if (!this.IsCurrentlyOpenToIntake)
    //         return;

    //     this.IsCurrentlyOpenToIntake = false;
    //     this.TargetAngle -= AddedAngleToRevolveOneStep / 2;
    // }

    public void IntakeModeOn() {
        if (this.IsCurrentlyOpenToIntake) return;

        this.IsCurrentlyOpenToIntake = true;
        this.TargetAngle += AddedAngleToRevolveOneStep / 2;
    }

    public void IntakeModeOff() {
        if (!this.IsCurrentlyOpenToIntake) return;

        this.IsCurrentlyOpenToIntake = false;
        this.TargetAngle -= AddedAngleToRevolveOneStep / 2;
    }

    public void SetPower(double Power) {
        this.Motor.setPower(Power);
    }

    public void Stop() {
        this.Motor.setPower(0);
    }
}
