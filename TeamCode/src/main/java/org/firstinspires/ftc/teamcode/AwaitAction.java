package org.firstinspires.ftc.teamcode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.Predicate;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

public class AwaitAction implements Action {
    BooleanSupplier Predicate;

    public AwaitAction(BooleanSupplier Predicate) {
        this.Predicate = Predicate;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        return !this.Predicate.getAsBoolean();
    }
}
