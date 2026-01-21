package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class AutoTest3Red extends LinearOpMode {
    @Override
    public void runOpMode() {
        Globals.telemetry = telemetry;

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        RobotAbstractor robot = new RobotAbstractor(hardwareMap);
        Intake intakeController = robot.IntakeSys;
        DecoderWheel decoderWheelController = robot.DecoderWheelSys;
        OutTake outTakeController = robot.OutTakeSys;

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException();
        }

        Pose2d beginPose = new Pose2d(1, 7, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        // MecanumDrive.SetShouldSeeBalls(true);
        TrajectoryActionBuilder MainActionBuilder = drive.actionBuilder(beginPose);

        AutoSteps StepsBuilder = new AutoSteps(MainActionBuilder, AutoSteps.TeamColor.RED, drive, robot);
        StepsBuilder.Init();

        Action MainActionChain = StepsBuilder.BuildAndGetActionBuilder();

        // Start the auto
        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        new UpdateAction(decoderWheelController::Update),
                        new UpdateAction(intakeController::Update),
                        new UpdateAction(outTakeController::Update),
                        new UpdateAction(this::TelemetryUpdate),
                        new SequentialAction(
                                MainActionChain
                        )
                )
        );
    }

    public void TelemetryUpdate(double DeltaTime) {
        telemetry.addData("should flip", "yes");
        telemetry.update();
    }
}