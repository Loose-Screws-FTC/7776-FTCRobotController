package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public enum BallOrder {
    GREEN1(new BallColor[3] {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE}),
    GREEN2(new BallColor[3] {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE}),
    GREEN3(new BallColor[3] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN}),
    IDK(new BallColor[3] {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE}),;

    public final BallColor[3] Colors;

    BallOrder(BallColor[3] colors) {
        Colors = colors;
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
