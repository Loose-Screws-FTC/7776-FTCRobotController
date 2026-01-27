package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class AutoBlueBack extends LinearOpMode {
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

        BackAutoSteps StepsBuilder = new BackAutoSteps(MainActionBuilder, TeamColor.BLUE, drive, robot);
        StepsBuilder.Init();

        Action autoActionSteps = StepsBuilder.BuildAndGetActionBuilder();

        // Start the auto
        waitForStart();
        Actions.runBlocking(autoActionSteps);
    }
}