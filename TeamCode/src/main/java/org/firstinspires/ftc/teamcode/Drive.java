package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeleOp1.LeftDistance;
import static org.firstinspires.ftc.teamcode.TeleOp1.RightDistance;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drive {
    public static Telemetry telemetry;

    private DcMotor FlMotor;
    private DcMotor FrMotor;
    private DcMotor BlMotor;
    private DcMotor BrMotor;

    private double FlSpeed;
    private double FrSpeed;
    private double BlSpeed;
    private double BrSpeed;

    private Gamepad Gamepad1;
    private Gamepad Gamepad2;

    private DriveState CurrentState = DriveState.IDLE;
    private double CurrentStateTimer;
    private boolean AprilTagTargetMode = false;
    private DriveMode CurrentMode = DriveMode.MANUAL;

    private double DriveSpeedMult = 1;
    private double RotSpeedMult = 1;

    public static double MaintainRotP = 1;
    public static double RotStoreSpeed = 0.05;

    private double CurrentRot;
    private double TargetRot = Double.NaN;

    private IMU Imu;

    private double limelightTx = 0;
    private boolean BallTargetMode;

    public void setLimelightTx(double tx) {
        this.limelightTx = tx;
    }

    public enum DriveState {
        IDLE,
        MOVING_IN_LOCAL_SPACE,
        MOVING_IN_GLOBAL_SPACE,
        MOVING_TO_POINT,
        E_STOP
    }

    public enum DriveMode {
        MANUAL,
        CONTROLLER_DRIVEN
    }

    public void Init(DcMotor FlDrive, DcMotor FrDrive, DcMotor BlDrive, DcMotor BrDrive, Gamepad Gamepad1, Gamepad Gamepad2, IMU Imu) {
        this.FlMotor = FlDrive;
        this.FrMotor = FrDrive;
        this.BlMotor = BlDrive;
        this.BrMotor = BrDrive;

        this.FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.FlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.FrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.FlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.FrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.BrMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.FlSpeed = 0;
        this.FrSpeed = 0;
        this.BlSpeed = 0;
        this.BrSpeed = 0;

        this.Gamepad1 = Gamepad1;
        this.Gamepad2 = Gamepad2;

        this.CurrentStateTimer = 0;


        IMU.Parameters IMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(new Orientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        AngleUnit.RADIANS,
                        0,
                        0,
                        (float)Math.PI / 4,
                        0
                ))
        );

        this.Imu = Imu;
        this.Imu.initialize(IMUParameters);
        this.Imu.resetYaw();

    }

    public void ResetIMU() {
        this.Imu.resetYaw();
    }

    public void Update(double DeltaTime) {
        this.CurrentStateTimer -= DeltaTime;

        CurrentRot = this.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (Double.isNaN(TargetRot)) {
            double yRotVel = this.Imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
            if (Math.abs(yRotVel) < RotStoreSpeed) {
                TargetRot = CurrentRot;
            }
        }

        if (this.Gamepad2.left_bumper) {
            this.CurrentState = DriveState.E_STOP;
            this.CurrentStateTimer = 1;
        }

        if (this.CurrentStateTimer <= 0) {
            this.CurrentStateTimer = 0;
            this.CurrentState = DriveState.IDLE;
        }

        if (this.CurrentState == DriveState.E_STOP) {
            Stop();
            return;
        }

        if (this.CurrentMode == DriveMode.CONTROLLER_DRIVEN) {
            Vector2 DriveDirection = new Vector2(Gamepad1.left_stick_x, Gamepad1.left_stick_y);
            DriveDirection.MultNum(-1);
            //DriveDirection.MultVector2(new Vector2(1, -1));
            DriveDirection.SquareVectWithSign(); // Square the input for a better speed curve
            DriveDirection.MultNum(DriveSpeedMult);

            if (Gamepad1.b) {
                DriveDirection.DivNum(2);
            }

            double turnAmount = 0;
            if (this.AprilTagTargetMode) {
                turnAmount = 0.025 * limelightTx;
                TargetRot = Double.NaN;
            }

            //Check if we are currently intaking and there is an object in front of one (not both) of the sensors.
            //Note the rev color/distance sensor v2 returns NaN when there is no object in fron of it, the v3 and other distance sensors behave very differently
            else if (this.BallTargetMode &&
                    !Double.isNaN(LeftDistance) && Double.isNaN(RightDistance)
            ) {
                Vector2 Vect = new Vector2(Math.sin(CurrentRot + Math.PI / 2), Math.cos(CurrentRot + Math.PI / 2));
                Vect.DivNum(3.5);
                DriveDirection.AddVector2(Vect);
            }

            else if (this.BallTargetMode &&
                    !Double.isNaN(RightDistance) && Double.isNaN(LeftDistance)
            ) {
                Vector2 Vect = new Vector2(Math.sin(CurrentRot - Math.PI / 2), Math.cos(CurrentRot - Math.PI / 2));
                Vect.DivNum(3.5);
                DriveDirection.AddVector2(Vect);
            } else if (Math.abs(Gamepad1.right_stick_x) > 0.01) {
                turnAmount = Gamepad1.right_stick_x * Gamepad1.right_stick_x * Math.signum(Gamepad1.right_stick_x) * RotSpeedMult;
                TargetRot = Double.NaN;
            } else if (!Double.isNaN(TargetRot)) {
                double TurnGoal = (CurrentRot - TargetRot + Math.PI) % (Math.PI * 2) - Math.PI;
                turnAmount = TurnGoal * MaintainRotP;
            }

            MoveInGlobalDirectionAndTurn(DriveDirection.GetNormal(), DriveDirection.GetMagnitude(), turnAmount, 1);
        }

        if (this.CurrentState != DriveState.IDLE) {
            this.FlMotor.setPower(FlSpeed);
            this.FrMotor.setPower(FrSpeed);
            this.BlMotor.setPower(BlSpeed);
            this.BrMotor.setPower(BrSpeed);
        } else {
            this.FlMotor.setPower(0);
            this.FrMotor.setPower(0);
            this.BlMotor.setPower(0);
            this.BrMotor.setPower(0);
        }
    }

    public void SetTargetingAprilTag(boolean isTargeting){
        this.AprilTagTargetMode = isTargeting;
    }

    public void SetTargetingBall(boolean isTargeting) {this.BallTargetMode = isTargeting;}

    public void SetDriveMode(DriveMode Mode) {
        this.CurrentMode = Mode;
    }

    /**
     * Moves the bot in local space for a desired time.
     * @param MoveDirection Direction that the bot will move.
     * @param Speed (Value 0-1) — The speed at which the bot will move.
     * @param TurnAmount How fast and in what direction the bot will turn.
     * @param Time How long the bot will move for.
     */
    public void MoveInLocalDirectionAndTurn(Vector2 MoveDirection, double Speed, double TurnAmount, double Time) {
        this.CurrentState = DriveState.MOVING_IN_LOCAL_SPACE;
        this.CurrentStateTimer = Time;

        FlSpeed = (MoveDirection.GetY() - MoveDirection.GetX()) * Speed + TurnAmount;
        FrSpeed = (MoveDirection.GetY() + MoveDirection.GetX()) * Speed - TurnAmount;
        BlSpeed = (MoveDirection.GetY() + MoveDirection.GetX()) * Speed + TurnAmount;
        BrSpeed = (MoveDirection.GetY() - MoveDirection.GetX()) * Speed - TurnAmount;
    }

    /**
     * Moves the bot in global space for a desired time.
     * @param MoveDirection Direction that the bot will move.
     * @param Speed (Value 0-1) — The speed at which the bot will move.
     * @param TurnAmount How fast and in what direction the bot will turn.
     * @param Time How long the bot will move for.
     */
    public void MoveInGlobalDirectionAndTurn(Vector2 MoveDirection, double Speed, double TurnAmount, double Time) {
        Vector2 LocalMoveDirection = new Vector2(
                MoveDirection.GetX() * Math.cos(CurrentRot) - MoveDirection.GetY() * Math.sin(CurrentRot),
                MoveDirection.GetX() * Math.sin(CurrentRot) + MoveDirection.GetY() * Math.cos(CurrentRot)
        );

        MoveInLocalDirectionAndTurn(LocalMoveDirection, Speed, TurnAmount, Time);
    }

    public void SetDriveSpeed(double Speed) {
        this.DriveSpeedMult = Speed;
    }

    public void SetRotationSpeed(double Speed) {
        this.RotSpeedMult = Speed;
    }

    /** Fully stops the robot */
    public void Stop() {
        this.FlMotor.setPower(0);
        this.FrMotor.setPower(0);
        this.BlMotor.setPower(0);
        this.BrMotor.setPower(0);
        this.FlSpeed = 0;
        this.FrSpeed = 0;
        this.BlSpeed = 0;
        this.BrSpeed = 0;
    }
}
