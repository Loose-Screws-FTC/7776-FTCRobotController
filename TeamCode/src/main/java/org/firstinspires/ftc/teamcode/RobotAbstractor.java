package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class RobotAbstractor {
    public static double BallColorTolerance = 0.006;
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

    private double BallDetectTime = Double.POSITIVE_INFINITY;
    private boolean BallDetected = false;
    private boolean RotatedAfterIntaking = false;

    public boolean ShouldIntake = false;
    public boolean CurrentlyIntaking = false;

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

    public void OutTakeBall() {
        this.OutTakeSys.ServosUp();
        this.DecoderWheelSys.ClearOuttakeSlot();
    }

    public void IntakeUpdate(double deltaTime) {
        NormalizedRGBA colors = ColorSensor.getNormalizedColors();
        if (Double.isInfinite(BallDetectTime)) {
            // Color sensor values typically float between 0.001 and 0.002 when looking at nothing,
            // and are normally between 0.01 and 0.03 for colored objects (depending on the color)
            if (colors.red > BallColorTolerance
                    || colors.green > BallColorTolerance
                    || colors.blue > BallColorTolerance) {
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
        IntakeSys.SetPower(IntakePower);
    }

    public void Update(double DeltaTime) {
        this.OutTakeSys.Update(DeltaTime);
        this.IntakeSys.Update(DeltaTime);
        this.DecoderWheelSys.Update(DeltaTime);
    }

    public void Stop() {
        this.OutTakeSys.Stop();
        this.IntakeSys.Stop();
    }
}
