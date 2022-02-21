package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
public class Slide {

    public DcMotor liftMotor = null;
    private LinearOpMode linearOpMode = null;
    public DigitalChannel digitalTouch_start;
    public DigitalChannel digitalTouch_end;
    public static double speed = 0.6;
    private boolean[] sensors = new boolean[3];

    public void init(LinearOpMode linearOpMode) {
        liftMotor = linearOpMode.hardwareMap.get(DcMotor.class, "Lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        digitalTouch_start = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        digitalTouch_start.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setMotorTarget(int direction, boolean extra) {
        sensors[0] = digitalTouch_start.getState();
        sensors[2] = digitalTouch_end.getState();
        if ((!sensors[1 + direction]) || (extra)) {
            liftMotor.setPower(speed * (double) direction);
            if (extra) liftMotor.setPower(1.0);
        }
        else{ liftMotor.setPower(0); }
    }
}
