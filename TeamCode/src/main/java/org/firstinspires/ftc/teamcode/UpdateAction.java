package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleConsumer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import static java.lang.System.currentTimeMillis;

import androidx.annotation.NonNull;

public class UpdateAction implements Action {
    double LastTime = currentTimeMillis() / 1000.0;
    DoubleConsumer Func;

    public UpdateAction(DoubleConsumer Func) {
        this.Func = Func;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double CurrTime = currentTimeMillis() / 1000.0;
        double DeltaTime = CurrTime - LastTime;
        LastTime = CurrTime;

        this.Func.accept(DeltaTime);

        return true;
    }
}
