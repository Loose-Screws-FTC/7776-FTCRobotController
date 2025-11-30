package org.firstinspires.ftc.teamcode;// package org.firstinspires.ftc.teamcode.Decode.teleop;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.Gamepad;
// 
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// 
// public class ClassicDrive {
//     public static Telemetry telemetry;
// 
//     private DcMotor FlMotor;
//     private DcMotor FrMotor;
//     private DcMotor BlMotor;
//     private DcMotor BrMotor;
// 
//     private double FlSpeed;
//     private double FrSpeed;
//     private double BlSpeed;
//     private double BrSpeed;
// 
//     private Gamepad Gamepad1;
//     private Gamepad Gamepad2;
// 
//     private DriveState CurrentState = DriveState.IDLE;
//     private double CurrentStateTimer;
// 
//     private int AutoStep = 0;
//     private double AutoStepTimer = 0;
// 
//     private DriveMode CurrentMode = DriveMode.MANUAL;
// 
//     public enum DriveState {
//         IDLE,
//         MOVING_IN_LOCAL_SPACE,
//         MOVING_IN_GLOBAL_SPACE,
//         MOVING_TO_POINT,
//         E_STOP
//     }
// 
//     public enum DriveMode {
//         MANUAL,
//         CONTROLLER_DRIVEN,
//         AUTO
//     }
// 
//     public void Init(DcMotor FlDrive, DcMotor FrDrive, DcMotor BlDrive, DcMotor BrDrive, Gamepad Gamepad1, Gamepad Gamepad2) {
//         this.FlMotor = FlDrive;
//         this.FrMotor = FrDrive;
//         this.BlMotor = BlDrive;
//         this.BrMotor = BrDrive;
// 
//         this.FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         this.FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         this.BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         this.BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 
//         this.FlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         this.FrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         this.BlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         this.BrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 
//         this.FlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//         this.FrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//         this.BlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//         this.BrMotor.setDirection(DcMotorSimple.Direction.FORWARD);
// 
//         this.FlSpeed = 0;
//         this.FrSpeed = 0;
//         this.BlSpeed = 0;
//         this.BrSpeed = 0;
// 
//         this.Gamepad1 = Gamepad1;
//         this.Gamepad2 = Gamepad2;
// 
//         this.CurrentStateTimer = 0;
//     }
// 
//     public void Update(double DeltaTime) {
//         this.CurrentStateTimer -= DeltaTime;
//         this.AutoStepTimer -= DeltaTime;
// 
//         if (this.Gamepad2.a) {
//             this.CurrentState = DriveState.E_STOP;
//             this.CurrentStateTimer = 1;
//         }
// 
//         if (this.CurrentStateTimer <= 0) {
//             this.CurrentStateTimer = 0;
//             this.CurrentState = DriveState.IDLE;
//         }
// 
//         if (this.CurrentState == DriveState.E_STOP) {
//             Stop();
//         }
// 
//         if (this.CurrentMode == DriveMode.CONTROLLER_DRIVEN) {
//             Vector2 DriveDirection = new Vector2(Gamepad1.left_stick_x, Gamepad1.left_stick_y);
//             DriveDirection.MultNum(-1);
//             DriveDirection.SquareVectWithSign(); // Square the input for a better speed curve
// 
//             if (Gamepad1.b) {
//                 DriveDirection.DivNum(2);
//             }
// 
//             MoveInLocalDirectionAndTurn(DriveDirection.GetNormal(), DriveDirection.GetMagnitude(), Gamepad1.right_stick_x * Gamepad1.right_stick_x * Math.signum(Gamepad1.right_stick_x), 1);
//         }
// 
//         if (this.CurrentMode == DriveMode.AUTO) {
//             Vector2 DriveDirection = new Vector2(0, 0);
// 
//             if (this.AutoStep == 1) {
//                 DriveDirection = new Vector2(0, 0.25);
//             } else if (this.AutoStep == 2) {
//                 DriveDirection = new Vector2(0, 0);
//             }
// 
//             if (this.AutoStepTimer <= 0) {
//                 if (this.AutoStep == 0) {
//                     this.AutoStepTimer = 1;
//                 }
// 
//                 if (this.AutoStep == 1) {
//                     this.AutoStepTimer = 23487;
//                 }
// 
//                 this.AutoStep += 1;
//             }
// 
//             MoveInLocalDirectionAndTurn(DriveDirection.GetNormal(), DriveDirection.GetMagnitude(), Gamepad1.right_stick_x * Gamepad1.right_stick_x * Math.signum(Gamepad1.right_stick_x), 1);
// 
//             telemetry.addData("Step", this.AutoStep);
//             telemetry.addData("Timer", this.AutoStepTimer);
//             telemetry.update();
//         }
// 
//         if (this.CurrentState != DriveState.IDLE) {
//             this.FlMotor.setPower(FlSpeed);
//             this.FrMotor.setPower(FrSpeed);
//             this.BlMotor.setPower(BlSpeed);
//             this.BrMotor.setPower(BrSpeed);
//         } else {
//             this.FlMotor.setPower(0);
//             this.FrMotor.setPower(0);
//             this.BlMotor.setPower(0);
//             this.BrMotor.setPower(0);
//         }
//     }
// 
//     public void SetDriveMode(DriveMode Mode) {
//         this.CurrentMode = Mode;
// 
//         if (Mode == DriveMode.AUTO) {
//             this.AutoStep = 0;
//             this.AutoStepTimer = 0;
//         }
//     }
// 
//     /**
//      * Moves the bot in local space for a desired time.
//      * @param MoveDirection Direction that the bot will move.
//      * @param Speed (Value 0-1) — The speed at which the bot will move.
//      * @param TurnAmount How fast and in what direction the bot will turn.
//      * @param Time How long the bot will move for.
//      */
//     public void MoveInLocalDirectionAndTurn(Vector2 MoveDirection, double Speed, double TurnAmount, double Time) {
//         this.CurrentState = DriveState.MOVING_IN_LOCAL_SPACE;
//         this.CurrentStateTimer = Time;
// 
//         FlSpeed = (MoveDirection.GetY() - MoveDirection.GetX()) * Speed + TurnAmount;
//         FrSpeed = (MoveDirection.GetY() + MoveDirection.GetX()) * Speed - TurnAmount;
//         BlSpeed = (MoveDirection.GetY() + MoveDirection.GetX()) * Speed + TurnAmount;
//         BrSpeed = (MoveDirection.GetY() - MoveDirection.GetX()) * Speed - TurnAmount;
//     }
// 
//     public double MoveToPoint(Vector2 Pos, double Rot, double Speed, double Precision) {
//         //double DirectionTo =
// 
//         return 0;
//     }
// 
//     /** Fully stops the robot */
//     public void Stop() {
//         this.FlMotor.setPower(0);
//         this.FrMotor.setPower(0);
//         this.BlMotor.setPower(0);
//         this.BrMotor.setPower(0);
//         this.FlSpeed = 0;
//         this.FrSpeed = 0;
//         this.BlSpeed = 0;
//         this.BrSpeed = 0;
//     }
// }
// 