package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class CollectBallAction implements Action {
    private final MecanumDrive Drive;
    public static double ForwardPower = 0.19;
    public static double SideSpeed = 0.3;
    public static double MaxRuntime = 5.0;
    private final double TargetDistance;
    private Vector2d InitialPos = null;
    private double StartTime = Double.NaN;

    public CollectBallAction(MecanumDrive drive, double forwardDistance) {
        this.Drive = drive;
        this.TargetDistance = forwardDistance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (Double.isNaN(StartTime)) {
            StartTime = currentTime;
        }

        double SidePower = 0;
        double LeftDistance = this.Drive.LeftDistanceSensor.getDistance(DistanceUnit.CM);
        double RightDistance = this.Drive.RightDistanceSensor.getDistance(DistanceUnit.CM);

        if (!Double.isNaN(LeftDistance)) {
            SidePower = -SideSpeed;
        } else if (!Double.isNaN(RightDistance)) {
            SidePower = SideSpeed;
        }

        this.Drive.leftFront.setPower(ForwardPower + SidePower);
        this.Drive.leftBack.setPower(ForwardPower - SidePower);
        this.Drive.rightFront.setPower(ForwardPower - SidePower);
        this.Drive.rightBack.setPower(ForwardPower + SidePower);

        this.Drive.updatePoseEstimate();

        Vector2d Pos = Drive.localizer.getPose().position;
        if (InitialPos == null) {
            InitialPos = Pos;
            return true;
        } else {
            Vector2d DeltaPos = Pos.minus(InitialPos);
            Globals.telemetry.addData("Dist", DeltaPos.norm());
            if (DeltaPos.sqrNorm() < TargetDistance * TargetDistance
                && (currentTime - StartTime) < MaxRuntime) {
                return true;
            } else {
                this.Drive.leftFront.setPower(0);
                this.Drive.leftBack.setPower(0);
                this.Drive.rightFront.setPower(0);
                this.Drive.rightBack.setPower(0);
                return false;
            }
        }
    }
}
