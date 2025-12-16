package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotAbstractor {
    public final OutTake OutTakeSys;
    public final Intake IntakeSys;
    public final DecoderWheel DecoderWheelSys;

    public final Limelight3A Limelight;

    public final NormalizedColorSensor ColorSensor;

    public final DistanceSensor LeftDistanceSensor;
    public final DistanceSensor RightDistanceSensor;

    public RobotAbstractor(HardwareMap hardwareMap) {
        DcMotorEx OutLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "outr");

        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");

        Servo TiltServo = hardwareMap.get(Servo.class, "tiltservo");

        this.OutTakeSys = new OutTake();
        this.OutTakeSys.Init(OutLeft, OutRight, OutLeftServo, OutRightServo, TiltServo);

        Servo InLeftServo = hardwareMap.get(Servo.class, "intakelefts");
        Servo InRightServo = hardwareMap.get(Servo.class, "intakerights");

        DcMotor InMotor = hardwareMap.get(DcMotor.class, "intake");

        this.IntakeSys = new Intake();
        this.IntakeSys.Init(InLeftServo, InRightServo, InMotor);

        DcMotor DecoderWheelMotor = hardwareMap.get(DcMotor.class, "ringdrive");

        this.DecoderWheelSys = new DecoderWheel();
        this.DecoderWheelSys.Init(DecoderWheelMotor);

        this.ColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        this.LeftDistanceSensor = hardwareMap.get(DistanceSensor.class,"lintakesensor");
        this.RightDistanceSensor = hardwareMap.get(DistanceSensor.class,"rintakesensor");



        this.Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.Limelight.setPollRateHz(100);
        this.Limelight.start();
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
