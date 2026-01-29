package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

@Config
public class RobotAbstractor {
    public static double BallColorTolerance = 0.0025;
    public static double BallDistanceThreshold = 4.75;
    public static double RetractIntakeTime = 0;
    public static double RevolveTime = 0.1;
    public static double RevolveFinishTime = 0.3;

    public final OutTake OutTakeSys;
    public final Intake IntakeSys;
    public final DecoderWheel DecoderWheelSys;

    public final Limelight3A Limelight;

    public final NormalizedColorSensor ColorSensor;

    public final DistanceSensor LeftDistanceSensor;
    public final DistanceSensor RightDistanceSensor;

    public final ElapsedTime Runtime;

    private List<Action> RunningActions = new ArrayList<>();

    private double BallDetectTime = Double.POSITIVE_INFINITY;
    private boolean RotatedAfterIntaking = false;

    public boolean BallDetected = false;
    public boolean ShouldIntake = false;
    public boolean CurrentlyIntaking = false;

    public boolean IntakeShouldOuttake = false;

    public int PatternOffset = 0;

    public RobotAbstractor(HardwareMap hardwareMap) {
        DcMotorEx OutLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "outr");

        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");

        Servo TiltServo = hardwareMap.get(Servo.class, "tiltservo");

        this.OutTakeSys = new OutTake();
        this.OutTakeSys.Init(OutLeft, OutRight, OutLeftServo, OutRightServo, TiltServo);

        this.ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        this.LeftDistanceSensor = hardwareMap.get(DistanceSensor.class,"lintakesensor");
        this.RightDistanceSensor = hardwareMap.get(DistanceSensor.class,"rintakesensor");

        Servo InLeftServo = hardwareMap.get(Servo.class, "intakelefts");
        Servo InRightServo = hardwareMap.get(Servo.class, "intakerights");

        DcMotor InMotor = hardwareMap.get(DcMotor.class, "intake");

        this.IntakeSys = new Intake();
        this.IntakeSys.Init(InLeftServo, InRightServo, InMotor);

        DcMotor DecoderWheelMotor = hardwareMap.get(DcMotor.class, "ringdrive");

        this.DecoderWheelSys = new DecoderWheel();
        this.DecoderWheelSys.Init(DecoderWheelMotor);

        this.Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.Limelight.setPollRateHz(100);
        this.Limelight.start();

        Runtime = new ElapsedTime();
    }

    public void AddAction(Action action) {
        RunningActions.add(action);
    }

    public BallColor GetNextBallColor() {
        return BallOrder.GameOrder.Colors[PatternOffset];
    }

    public void ConsumeBallColor() {
        PatternOffset = (PatternOffset + 1) % 3;
    }

    public void UnconsumeBallColor() {
        if (PatternOffset-- == 0) {
            PatternOffset = 2;
        }
    }

    public void ResetMotifProgress() {
        PatternOffset = 0;
    }

    public void StartOutTakeBall() {
        this.OutTakeSys.ServosUp();
        BallColor occupant = this.DecoderWheelSys.ClearOuttakeSlot();
        if (occupant.IsBall) {
            ConsumeBallColor();
        }
    }

    public void AutoIntakeUpdate(double deltaTime) {
        NormalizedRGBA colors = ColorSensor.getNormalizedColors();
        if (Double.isInfinite(BallDetectTime)) {
            // Color sensor values typically float between 0.001 and 0.002 when looking at nothing,
            // and are normally between 0.01 and 0.03 for colored objects (depending on the color)
            boolean colorsMet = colors.red > BallColorTolerance
                    || colors.green > BallColorTolerance
                    || colors.blue > BallColorTolerance;
            boolean distanceMet = ((DistanceSensor)ColorSensor).getDistance(DistanceUnit.CM) < BallDistanceThreshold;
            if (colorsMet && distanceMet) {
                DecoderWheelSys.SetIntakedColor(DecoderWheel.DetermineBallColor(colors));
                this.BallDetected = true;
                this.BallDetectTime = Runtime.seconds();
            } else {
                this.BallDetected = false;
            }
        }

        CurrentlyIntaking = ShouldIntake;

        if (Runtime.seconds() > BallDetectTime + RevolveFinishTime) {
            this.BallDetectTime = Double.POSITIVE_INFINITY;
        }
        if (Runtime.seconds() > BallDetectTime + RevolveTime) {
            if (!RotatedAfterIntaking) {
                RotatedAfterIntaking = true;
                DecoderWheelSys.RevolveRight();
            }
            CurrentlyIntaking = false;
        } else if (Runtime.seconds() > BallDetectTime + RetractIntakeTime) {
            RotatedAfterIntaking = false;
            CurrentlyIntaking = false;
        }

        if (CurrentlyIntaking) {
            IntakeSys.ServosToIntake();
            DecoderWheelSys.IntakeModeOn();
        } else {
            IntakeSys.ServosToNeutral();
            DecoderWheelSys.IntakeModeOff();
        }

        double IntakePower = CurrentlyIntaking || !DecoderWheelSys.IsAtTarget()
                ? 1.0 : 0.0;
        if (!IntakeShouldOuttake) {
            IntakeSys.SetPower(IntakePower);
        } else {
            IntakeSys.SetPower(-1);
        }
    }

    public void Update(double DeltaTime) {
        this.OutTakeSys.Update(DeltaTime);
        this.IntakeSys.Update(DeltaTime);
        this.DecoderWheelSys.Update(DeltaTime);
        UpdateRunningActions();
    }

    private void UpdateRunningActions() {
        TelemetryPacket packet = new TelemetryPacket();
        RunningActions.removeIf(action -> !action.run(packet));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void Stop() {
        this.OutTakeSys.Stop();
        this.IntakeSys.Stop();
    }

    public Action ShootOneBallAction() {
        double RPMStabilizeTime = 0.2;
        double ServoUpTime = 0.3;
        double ServoDownTime = 0.3;
        return new SequentialAction(
            new AwaitAction(DecoderWheelSys::IsAtTarget),
            new AwaitAction(OutTakeSys::IsAtVelocity),
            new SleepAction(RPMStabilizeTime),
            new AwaitAction(OutTakeSys::IsAtVelocity),
            new InstantAction(this::StartOutTakeBall),
            new SleepAction(ServoUpTime),
            new InstantAction(OutTakeSys::ServosDown),
            new SleepAction(ServoDownTime)
        );
    }

    public Action ShootAllBallsAction() {
        return new SequentialAction(
            new InstantAction(DecoderWheelSys::AddDummyBalls),
            new InstantAction(() -> IntakeSys.SetPower(1)),
            new InstantAction(IntakeSys::ServosToNeutral),
            new SleepAction(0.1), // wait for the intake servos to move
            new InstantAction(() -> DecoderWheelSys.RevolveToColor(GetNextBallColor())),
            new WaitOneFrameAction(),
            ShootOneBallAction(),
            new InstantAction(() -> DecoderWheelSys.RevolveToColor(GetNextBallColor())),
            new WaitOneFrameAction(),
            ShootOneBallAction(),
            new InstantAction(() -> DecoderWheelSys.RevolveToColor(GetNextBallColor())),
            new WaitOneFrameAction(),
            ShootOneBallAction(),
            new InstantAction(() -> IntakeSys.SetPower(0))
        );
    }
}
