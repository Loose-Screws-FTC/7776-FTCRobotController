package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

@Config
public class BackAutoSteps extends AutoSteps {
    TrajectoryActionBuilder CurrentActionBuilder;
    Intake IntakeController;
    OutTake OutTakeController;
    DecoderWheel DecoderWheelController;
    TeamColor BaseColor = TeamColor.RED;
    TeamColor AllianceColor;

    public static double LaunchRPM = 1750;

    public static double FireTime = 10;

    public static boolean Fire3Balls = true;

    public BackAutoSteps(TrajectoryActionBuilder ActionBuilder, TeamColor AllianceColor, MecanumDrive Drive, RobotAbstractor robot) {
        super(Drive, robot);
        CurrentActionBuilder = ActionBuilder;
        this.AllianceColor = AllianceColor;
        this.ShouldFlip = BaseColor != AllianceColor;
        this.Drive = Drive;
        this.Robot = robot;
        this.IntakeController = robot.IntakeSys;
        this.OutTakeController = robot.OutTakeSys;
        this.DecoderWheelController = robot.DecoderWheelSys;
    }

    @Override
    public void Init() {
        this.DecoderWheelController.SetCurrentColors(
            BallColor.GREEN,
            BallColor.PURPLE,
            BallColor.PURPLE
        );
    }

    @Override
    public Action BuildAndGetActionBuilder() {
        // https://www.desmos.com/calculator/xrp25mvpxb
        TrajectoryActionBuilder builder = CurrentActionBuilder;

        if (Fire3Balls) {
            builder = builder
                .stopAndAdd(() -> OutTakeController.SetVelocity(LaunchRPM / 6000.0))
                .stopAndAdd(() -> OutTakeController.SetIsFiring(true));
        }
        
        builder = builder
            .splineToLinearHeading(MapPose(new Pose2d(0, 8, Math.toRadians(0))), 0)
            .turn(MapAngle(Math.toRadians(-95)))
            .stopAndAdd(new FindBallOrderAction(Robot, 5))
            .stopAndAdd(() -> Robot.RecordMotifOffset())
            .stopAndAdd(() -> DecoderWheelController.RevolveToColor(Robot.GetMotifBallColor(0)));
        
        if (Fire3Balls) {
            builder = builder
                .turn(MapAngle(Math.toRadians(-19.5)))
                .stopAndAdd(new AwaitAction(() -> Robot.Runtime.seconds() > FireTime))
                .stopAndAdd(Robot.ShootAllBallsAction());
        }

        builder = builder
            .splineToLinearHeading(MapPose(new Pose2d(20, 8, Math.toRadians(0))), 0);
        
        Action steps = builder.build();

        return new ParallelAction(
            new UpdateAction(Robot::Update),
            new UpdateAction(Robot::AutoIntakeUpdate),
            new UpdateAction(this::TelemetryUpdate),
            new SequentialAction(
                steps
            )
        );
    }

}
