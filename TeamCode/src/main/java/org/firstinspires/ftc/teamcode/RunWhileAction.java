package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import java.util.function.BooleanSupplier;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

@Config
public class RunWhileAction implements Action {
    BooleanSupplier ShouldContinue;
    Action Action;

    public FindBallOrderAction(BooleanSupplier shouldContinue, Action Action) {
        this.ShouldContinue = shouldContinue;
        this.Action = Action;
    }

    public boolean run(@NonNull TelemetryPacket packet) {
        if (!ShouldContinue.getAsBoolean()) {
            return false;
        }
        return this.Action.run(packet);
    }
}
