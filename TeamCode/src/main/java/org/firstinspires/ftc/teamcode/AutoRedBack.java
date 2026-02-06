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
public class AutoRedBack extends LinearOpMode {
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

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        // MecanumDrive.SetShouldSeeBalls(true);
        TrajectoryActionBuilder MainActionBuilder = drive.actionBuilder(beginPose);

        BackAutoSteps StepsBuilder = new BackAutoSteps(MainActionBuilder, TeamColor.RED, drive, robot);
        StepsBuilder.Init();

        Action autoActionSteps = StepsBuilder.BuildAndGetActionBuilder();

        // Start the auto
        waitForStart();
        StepsBuilder.FastRunAction(autoActionSteps);
    }
}