package org.firstinspires.ftc.teamcode;

public enum BallColor {
    GREEN(true),
    PURPLE(true),
    DUMMY(false),
    NONE(false);

    public boolean IsBall;

    BallColor(boolean isBall) {
        IsBall = isBall;
    }
}
