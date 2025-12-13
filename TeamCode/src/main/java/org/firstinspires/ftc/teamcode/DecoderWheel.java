package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class DecoderWheel {
    private DcMotorEx Motor;

    public static int IntakeSlot = 1;

    private double AddedAngleToRevolveOneStep = 120;
    private double TicksPerRev = 28 * 6.2;
    private double TargetAngle = 0;
    private double CurrAngle = 0;

    public static double MaxMotorPower = 0.5;
    public static double CloseMotorPower = 0.2;
    public static double AcceptableAngleDeviation = 5;
    public static double CloseAngleDeviation = 15;

    private boolean IsCurrentlyOpenToIntake = false;

    private boolean IsAtTarget = false;

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
//        this.Motor.setTargetPosition(0);
//        this.Motor.setPower(MotorPower);
//        this.Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        this.CurrAngle = this.Motor.getCurrentPosition() / TicksPerRev * 360;

        boolean isClose = Math.abs(this.TargetAngle - this.CurrAngle) < CloseAngleDeviation;
        double baseMotorPower = isClose ? CloseMotorPower : MaxMotorPower;

        if (this.CurrAngle < this.TargetAngle - AcceptableAngleDeviation) {
            this.Motor.setPower(baseMotorPower);
            IsAtTarget = false;
        } else if (this.CurrAngle > this.TargetAngle + AcceptableAngleDeviation) {
            this.Motor.setPower(-baseMotorPower);
            IsAtTarget = false;
        } else {
            this.Motor.setPower(0);
            IsAtTarget = true;
        }

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

    public boolean GetIsAtTarget() {
        return this.IsAtTarget;
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

    public void SetIntakedColor(BallColor Color) {
        BallsInWheel.set(IntakeSlot, Color);
    }

    public void ClearOuttakeSlot() {
        BallsInWheel.set(0, BallColor.NONE);
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

        int IndexOfColor = -1;

        if (this.BallsInWheel.get(0) == Color) {
            return true;
        } else if (this.BallsInWheel.get(1) == Color) {
            IndexOfColor = 1;
        } else if (this.BallsInWheel.get(2) == Color) {
            IndexOfColor = 2;
        }

        if (IndexOfColor == -1) {
            return false;
        }

        // Index will always either be 1 or 2
        if (IndexOfColor == 1) {
            this.RevolveLeft();
        } else {
            this.RevolveRight();
        }

        return true;
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
