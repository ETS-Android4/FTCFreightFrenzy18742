package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Lift {

    private DcMotor liftMotor = null;
    private LinearOpMode linearOpMode = null;
    public DigitalChannel digitalTouch_start;
    public DigitalChannel digitalTouch_end;
    public double speed = 1;
    private boolean [] sensors = {true,true,true};

    public void init(LinearOpMode linearOpMode) {
        liftMotor = linearOpMode.hardwareMap.get(DcMotor.class, "Lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        digitalTouch_start = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        digitalTouch_start.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setMotorTarget(int direction) {
        sensors[0] = digitalTouch_start.getState();
        sensors[2] = digitalTouch_end.getState();
        while(sensors[1 + direction]){
            liftMotor.setPower(speed * direction);
        }
        liftMotor.setPower(0);
    }

}
