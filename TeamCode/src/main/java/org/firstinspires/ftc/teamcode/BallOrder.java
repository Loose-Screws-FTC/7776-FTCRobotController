package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public enum BallOrder {
    GREEN1(DecoderWheel.BallColor.GREEN, DecoderWheel.BallColor.PURPLE, DecoderWheel.BallColor.PURPLE),
    GREEN2(DecoderWheel.BallColor.PURPLE, DecoderWheel.BallColor.GREEN, DecoderWheel.BallColor.PURPLE),
    GREEN3(DecoderWheel.BallColor.PURPLE, DecoderWheel.BallColor.PURPLE, DecoderWheel.BallColor.GREEN),
    IDK(DecoderWheel.BallColor.GREEN, DecoderWheel.BallColor.PURPLE, DecoderWheel.BallColor.PURPLE),;

    public final DecoderWheel.BallColor Ball1;
    public final DecoderWheel.BallColor Ball2;
    public final DecoderWheel.BallColor Ball3;

    BallOrder(DecoderWheel.BallColor ball1, DecoderWheel.BallColor ball2, DecoderWheel.BallColor ball3) {
        Ball1 = ball1;
        Ball2 = ball2;
        Ball3 = ball3;
    }

    public static BallOrder GameOrder;

    static BallOrder FindBallOrder(RobotAbstractor robot) {
        LLResult llResult = robot.Limelight.getLatestResult();
        if (llResult == null) return BallOrder.IDK;
        BallOrder order = BallOrder.IDK;
        for (LLResultTypes.FiducialResult fiducialResult : llResult.getFiducialResults()) {
            int id = fiducialResult.getFiducialId();
            if (id == 21) {
                order = BallOrder.GREEN1;
            } else if (id == 22) {
                order = BallOrder.GREEN2;
            } else if (id == 23) {
                order = BallOrder.GREEN3;
            }
        }
        return order;
    }

    public static void DetectBallOrder(RobotAbstractor robot) {
        if (GameOrder != IDK) {
            GameOrder = FindBallOrder(robot);
        }
    }
}
