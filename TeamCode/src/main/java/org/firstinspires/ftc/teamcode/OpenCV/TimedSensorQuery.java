package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class TimedSensorQuery<T>{

    private Supplier<T> sensorValueSupplier = null;

    private T lastValue = null;

    private double updateTimeSeconds;

    private final ElapsedTime queryTimer = new ElapsedTime();

    public TimedSensorQuery(Supplier<T> sensorValueSupplier, double refreshRateHz) {
        this.sensorValueSupplier = sensorValueSupplier;
        updateTimeSeconds = 1 / refreshRateHz;
    }

    public TimedSensorQuery(Supplier<T> sensorValueSupplier) {
        this(sensorValueSupplier, 100);
    }

    public T getValue() {
        if (lastValue == null)
            lastValue = sensorValueSupplier.get();
        else if (queryTimer.seconds() > updateTimeSeconds) {
            lastValue = sensorValueSupplier.get();
            queryTimer.reset();
        }
        return lastValue;
    }
}
