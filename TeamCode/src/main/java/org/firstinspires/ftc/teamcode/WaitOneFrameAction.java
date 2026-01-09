package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.Predicate;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

public class WaitOneFrameAction implements Action {
    boolean waited = false;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (waited) {
            return false;
        } else {
            waited = true;
            return true;
        }
    }
}
