package org.firstinspires.ftc.teamcode.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
public class TouchSystem {

    private LinearOpMode linearOpMode = null;
    public DigitalChannel digitalTouch_start;
    public DigitalChannel digitalTouch_end;

    public void init(LinearOpMode linearOpMode) {
        digitalTouch_start = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        digitalTouch_start.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getTouch(int position) {
        if (position == 0) { return !digitalTouch_start.getState(); }
        else { return !digitalTouch_end.getState(); }
    }
}