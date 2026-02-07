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

import java.io.BufferedReader;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.List;

@Config
public class RobotAbstractor {
    public static double BallColorTolerance = 0.006;
    public static double BallDistanceThreshold = 15;
    public static int FactorsRequired = 1;
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

    public boolean IntakeShouldOuttake = false;

    public int ClassifierBallsHeld = 0;
    public int MotifOffset = 0;

    public static final int CLASSIFIER_CAP = 9;

    public RobotAbstractor(HardwareMap hardwareMap, boolean shouldZero) {
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
        this.DecoderWheelSys.Init(DecoderWheelMotor, shouldZero);

        this.Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.Limelight.setPollRateHz(100);
        this.Limelight.start();

        Runtime = new ElapsedTime();
    }

    public void ToStartPositions() {
        this.OutTakeSys.ServosDown();
        this.IntakeSys.ServosToNeutral();
    }

    public void AddAction(Action action) {
        RunningActions.add(action);
    }

    // record the current number of balls in the classifier as the offset for the motif
    // (if another ball is added to the classifier while the motif is still running,
    // it won't affect the motif. this can be both a plus and a minus. if the ball is from
    // the motif being shot by the robot itself, then it would be entirely incorrect to have it
    // affect the motif progress. if a ball actually was shot into the classifier by our alliance
    // partner before or during the motif by this robot, then accounting for it would allow us
    // to score more pattern points. there's no way to (easily) tell which case it is, so we must
    // ignore all balls recorded in the classifier after the motif starts)
    public void RecordMotifOffset() {
        MotifOffset = ClassifierBallsHeld;
    }

    public BallColor GetMotifBallColor(int nth) {
        return BallOrder.GameOrder.Colors[(MotifOffset + nth) % 3];
    }

    public void ShowClassifierBallCountInTelemetry() {
        // these are rendered on the telemetry output of the driver station in Roboto
        String[] arts = {
            "     ⬜\n" +
            "⬜     ⬜\n" +
            "⬜     ⬜\n" +
            "⬜     ⬜\n" +
            "     ⬜",

            "     ⬜\n" +
            "⬜⬜\n" +
            "     ⬜\n" +
            "     ⬜\n" +
            "⬜⬜⬜",

            "  ⬜⬜\n" +
            "          ⬜\n" +
            "       ⬜\n" +
            "  ⬜\n" +
            "⬜⬜⬜",

            "⬜⬜⬜\n" +
            "          ⬜\n" +
            "⬜⬜⬜\n" +
            "          ⬜\n" +
            "⬜⬜⬜",

            "⬜     ⬜\n" +
            "⬜     ⬜\n" +
            "⬜⬜⬜\n" +
            "          ⬜\n" +
            "          ⬜",

            "⬜⬜⬜\n" +
            "⬜\n" +
            "⬜⬜\n" +
            "          ⬜\n" +
            "⬜⬜",

            "     ⬜⬜\n" +
            "⬜\n" +
            "⬜⬜⬜\n" +
            "⬜     ⬜\n" +
            "⬜⬜⬜ ",

            "⬜⬜⬜\n" +
            "         ⬜\n" +
            "       ⬜\n" +
            "   ⬜\n" +
            "⬜",

            "⬜⬜⬜\n" +
            "⬜     ⬜\n" +
            "⬜⬜⬜\n" +
            "⬜     ⬜\n" +
            "⬜⬜⬜",

            "⬜⬜⬜\n" +
            "⬜     ⬜\n" +
            "⬜⬜⬜\n" +
            "          ⬜\n" +
            "⬜⬜"
        };
        String art = arts[ClassifierBallsHeld];
        for (String line : art.split("\\n")) {
            Globals.telemetry.addLine(line);
        }
    }

    public void AddToClassifier() {
        ClassifierBallsHeld = (ClassifierBallsHeld + 1) % CLASSIFIER_CAP;
    }

    public void RemoveFromClassifier() {
        if (ClassifierBallsHeld-- == 0) {
            ClassifierBallsHeld = CLASSIFIER_CAP - 1;
        }
    }

    public void ClearClassifier() {
        ClassifierBallsHeld = 0;
    }

    public void StartOutTakeBall() {
        this.OutTakeSys.ServosUp();
        this.DecoderWheelSys.ClearOuttakeSlot();
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
            int factorsPresent = (colorsMet ? 1 : 0) + (distanceMet ? 1 : 0);
            if (factorsPresent >= FactorsRequired) {
                DecoderWheelSys.SetIntakedColor(DecoderWheel.DetermineBallColor(colors));
                this.BallDetected = true;
                this.BallDetectTime = Runtime.seconds();
            } else {
                this.BallDetected = false;
            }
        }

        boolean CurrentlyIntaking = ShouldIntake;

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
        if (IntakeShouldOuttake) {
            DecoderWheelSys.ClearIntakedSlot();
            IntakeSys.SetPower(-1);
        } else {
            IntakeSys.SetPower(IntakePower);
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
            new InstantAction(this::RecordMotifOffset),
            new InstantAction(DecoderWheelSys::AddDummyBalls),
            new InstantAction(() -> IntakeSys.SetPower(1)),
            new InstantAction(IntakeSys::ServosToNeutral),
            new SleepAction(0.1), // wait for the intake servos to move
            new InstantAction(() -> DecoderWheelSys.RevolveToColor(GetMotifBallColor(0))),
            new WaitOneFrameAction(),
            ShootOneBallAction(),
            new InstantAction(() -> DecoderWheelSys.RevolveToColor(GetMotifBallColor(1))),
            new WaitOneFrameAction(),
            ShootOneBallAction(),
            new InstantAction(() -> DecoderWheelSys.RevolveToColor(GetMotifBallColor(2))),
            new WaitOneFrameAction(),
            ShootOneBallAction(),
            new InstantAction(() -> IntakeSys.SetPower(0))
        );
    }
}
