package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public enum BallOrder {
    GREEN1(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),
    GREEN2(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
    GREEN3(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN),
    IDK(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),;

    public final BallColor Ball1;
    public final BallColor Ball2;
    public final BallColor Ball3;

    BallOrder(BallColor ball1, BallColor ball2, BallColor ball3) {
        Ball1 = ball1;
        Ball2 = ball2;
        Ball3 = ball3;
    }

    public static BallOrder GameOrder = IDK;

    public static BallOrder FindBallOrder(RobotAbstractor robot) {
        LLResult llResult = robot.Limelight.getLatestResult();
        if (llResult == null) return BallOrder.IDK;
        BallOrder order = BallOrder.IDK;
        double bestBallOrderX = Double.POSITIVE_INFINITY;
        for (LLResultTypes.FiducialResult fiducialResult : llResult.getFiducialResults()) {
            int id = fiducialResult.getFiducialId();
            BallOrder orderHere;
            if (id == 21) {
                orderHere = BallOrder.GREEN1;
            } else if (id == 22) {
                orderHere = BallOrder.GREEN2;
            } else if (id == 23) {
                orderHere = BallOrder.GREEN3;
            } else {
                continue;
            }
            if (fiducialResult.getTargetXPixels() < bestBallOrderX) {
                bestBallOrderX = fiducialResult.getTargetXPixels();
                order = orderHere;
            }
        }
        return order;
    }

    public static void DetectBallOrder(RobotAbstractor robot) {
        BallOrder order = FindBallOrder(robot);
        if (order != IDK) {
            GameOrder = order;
        }
    }
}
