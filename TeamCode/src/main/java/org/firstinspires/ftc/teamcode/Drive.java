package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeleOp1.LeftDistance;
import static org.firstinspires.ftc.teamcode.TeleOp1.RightDistance;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    private Odometry OdometryController;

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

    public void Init(DcMotor FlDrive, DcMotor FrDrive, DcMotor BlDrive, DcMotor BrDrive, Gamepad Gamepad1, Gamepad Gamepad2, IMU Imu2) {
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

        this.OdometryController = new Odometry();
        this.OdometryController.Init(this.BlMotor, this.FrMotor , Imu2);

    }

    public void ResetIMU() {
        this.OdometryController.ResetIMU();
    }

    public void Update(double DeltaTime) {
        this.CurrentStateTimer -= DeltaTime;

        this.OdometryController.Update(DeltaTime);

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
            }

            else if (this.BallTargetMode &&
                    !Double.isNaN(LeftDistance)
            ) {
                turnAmount = -0.3;
            }

            else if (this.BallTargetMode &&
                    !Double.isNaN(RightDistance)
            ) {
                turnAmount = 0.3;
            }

            else {
                turnAmount = Gamepad1.right_stick_x * Gamepad1.right_stick_x * Math.signum(Gamepad1.right_stick_x) * RotSpeedMult;
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
        double Rot = this.OdometryController.GetHeading();

        Vector2 LocalMoveDirection = new Vector2(
                MoveDirection.GetX() * Math.cos(Rot) - MoveDirection.GetY() * Math.sin(Rot),
                MoveDirection.GetX() * Math.sin(Rot) + MoveDirection.GetY() * Math.cos(Rot)
        );

        MoveInLocalDirectionAndTurn(LocalMoveDirection, Speed, TurnAmount, Time);

        return;
    }

    public double MoveToPoint(Vector2 Pos, double Rot, double Speed, double Precision) {
        //double DirectionTo =

        return 0;
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
