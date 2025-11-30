package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Inspection", group="Iterative Opmode")
public class Inspection extends OpMode {
    private Intake IntakeController;

    @Override
    // Has to be lowercase init()
    public void init() {
        Intake.telemetry = telemetry;

        Servo InLeftServo = hardwareMap.get(Servo.class, "intakelefts");
        Servo InRightServo = hardwareMap.get(Servo.class, "intakerights");

        DcMotor InMotor = hardwareMap.get(DcMotor.class, "intake");

        this.IntakeController = new Intake();
        this.IntakeController.Init(InLeftServo, InRightServo, InMotor);
    }

    // Has to be lowercase loop()
    double LastRecTime = currentTimeMillis();
    public void loop() {
        long CurrTime = currentTimeMillis();
        double DeltaTime = (CurrTime / 1000.0) - LastRecTime;
        LastRecTime = CurrTime / 1000.0;

        this.IntakeController.Update(DeltaTime);

        this.IntakeController.ServosToIntake();
    }

    public void stop() {
        this.IntakeController.Stop();
    }
}