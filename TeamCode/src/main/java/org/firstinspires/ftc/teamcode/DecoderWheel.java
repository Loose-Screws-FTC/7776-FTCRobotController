package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class DecoderWheel {
    private DcMotorEx Motor;

    private final int IntakeSlot = 1;

    private final double AddedAngleToRevolveOneStep = 120;
    private final double TicksPerRev = 560;
    private static double TargetAngle = 0;
    private static double CurrAngle = 0;

//    public static double CloseMotorPower = 0.2;
    public static double[] BallsPowerMultiplier = {
        0.5,
        0.65,
        0.75,
        1
    };
    public static double AcceptableAngleDeviation = 5;
//    public static double CloseAngleDeviation = 30;

    public static double PMaxAngle = 50;
    public static double PMinAngle = 5;
    public static double MinMotorPower = 0.2;
    public static double MaxMotorPower = 1;

    private boolean IsCurrentlyOpenToIntake = false;

    private boolean AtTarget = false;

    // Index 0 should always be the one in front of the outtake wheels.
    private List<BallColor> BallsInWheel = new ArrayList<>();

    public void Init(DcMotor WheelMotor, boolean shouldZero) {
        this.Motor = (DcMotorEx)WheelMotor;

        this.Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (shouldZero) {
            this.Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TargetAngle = 0;
            CurrAngle = 0;
        }
        this.Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        this.Motor.setTargetPosition(0);
//        this.Motor.setPower(MaxMotorPower);
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

        Globals.telemetry.addData("ball count", this.GetBallCount());
        Globals.telemetry.addData("deltaTime", DeltaTime);

        Globals.telemetry.addData("decoder wheel RPM", this.Motor.getVelocity() / TicksPerRev * 60);

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

        double angleDeviation = Math.abs(this.TargetAngle - this.CurrAngle);
        double interPDist = (angleDeviation - PMinAngle) / (PMaxAngle - PMinAngle);
        if (interPDist < 0) interPDist = 0;
        if (interPDist > 1) interPDist = 1;
        double baseMotorPower = interPDist * MaxMotorPower + (1 - interPDist) * MinMotorPower;
        // boolean isClose = angleDeviation < CloseAngleDeviation;
        // double baseMotorPower = isClose ? CloseMotorPower : MaxMotorPower;
        baseMotorPower *= BallsPowerMultiplier[GetBallCount()];

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("curr angle", this.CurrAngle);
        packet.put("target angle", this.TargetAngle);
        packet.put("baseMotorPower", baseMotorPower * 100);
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
//        this.Motor.setPositionPIDFCoefficients(PosP);
//        this.Motor.setVelocityPIDFCoefficients(
//            VelP,
//            VelI,
//            VelD,
//            VelF
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

    public int GetBallCount() {
        int count = 0;
        for (BallColor color : BallsInWheel) {
            if (color.IsBall) count++;
        }
        return count;
    }

    public void SetIntakedColor(BallColor Color) {
        BallsInWheel.set(IntakeSlot, Color);
    }

    public void ClearIntakedSlot() {
        BallsInWheel.set(IntakeSlot, BallColor.NONE);
    }

    public BallColor ClearOuttakeSlot() {
        return BallsInWheel.set(0, BallColor.NONE);
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

    public void RevolveToColor(BallColor Color) {
        if (this.IsCurrentlyOpenToIntake)
            return;

        if (this.BallsInWheel.get(0) == Color) return;

        if (this.BallsInWheel.get(1) == Color) {
            this.RevolveLeft();
        } else if (this.BallsInWheel.get(2) == Color) {
            this.RevolveRight();
        } else {
            RevolveToAnyColor();
        }
    }

    public void RevolveToAnyColor() {
        if (this.BallsInWheel.get(0) != BallColor.NONE) return;

        if (this.BallsInWheel.get(1) != BallColor.NONE) {
            this.RevolveLeft();
        } else if (this.BallsInWheel.get(2) != BallColor.NONE) {
            this.RevolveRight();
        }
    }

    // Adds purple balls to all empty slots
    public void AddDummyBalls() {
        for (int i = 0; i < 3; i++) {
            if (this.BallsInWheel.get(i) == BallColor.NONE) {
                this.BallsInWheel.set(i, BallColor.DUMMY);
            }
        }
    }

    public void ManualAdjustAngle(double offset) {
        TargetAngle += offset;
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
}
