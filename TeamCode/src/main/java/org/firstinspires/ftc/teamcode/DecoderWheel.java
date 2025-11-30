package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class DecoderWheel {
    public static Telemetry telemetry;

    private DcMotor Motor;

    private double AddedAngleToRevolveOneStep = 120;
    private double TicksPerRev = 28 * 6.2;
    private double TargetAngle = 0;
    private double CurrAngle = 0;
    private double MotorPower = 0.5;

    private boolean IsCurrentlyOpenToIntake = false;

    private Intake IntakeController;

    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    // Index 0 should always be the one in front of the outtake wheels.
    private List<BallColor> BallsInWheel = new ArrayList<>();

    public void Init(DcMotor WheelMotor) {
        this.Motor = WheelMotor;

        this.Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.BallsInWheel.add(BallColor.NONE);
        this.BallsInWheel.add(BallColor.NONE);
        this.BallsInWheel.add(BallColor.PURPLE);
    }

    public void Update(double DeltaTime) {
        this.CurrAngle = this.Motor.getCurrentPosition() / TicksPerRev * 360;

        boolean OffTarget = false;

        if (this.CurrAngle < this.TargetAngle - 5) {
            this.Motor.setPower(MotorPower);
            OffTarget = true;
        } else if (this.CurrAngle > this.TargetAngle + 5) {
            this.Motor.setPower(-MotorPower);
            OffTarget = true;
        } else {
            this.Motor.setPower(0);
        }

        if (OffTarget) {
            this.IntakeController.SetPower(1);
        }

        telemetry.addData("Current angle", this.CurrAngle);
        telemetry.addData("Motor", this.Motor.getCurrentPosition());
    }

    /*
    public void SetAngle(double Angle) {
        this.CurrentAngle = Angle;
        int TargetTicks = (int)(CurrentAngle / 360.0 * TicksPerRev);
        this.Motor.setTargetPosition(TargetTicks);
        this.Motor.setPower(0.5);
    }
     */

    public void SetIntake(Intake In) {
        this.IntakeController = In;
    }

    public void RevolveRight() {
        this.TargetAngle += AddedAngleToRevolveOneStep;

        Collections.rotate(BallsInWheel, 1);
    }

    public void RevolveLeft() {
        this.TargetAngle -= AddedAngleToRevolveOneStep;

        Collections.rotate(BallsInWheel, -1);
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
