package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Sensors.TouchSystem;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpSticks;

@Config
public class Bucket {
    public boolean optical = true;
    public static double position1 = 0.15d;
    public static double position2 = 0.5d;
    public static double servoSpeedSeconds = 0.1d;
    private ElapsedTime blockTimer = new ElapsedTime();
    DigitalChannel digitalTouch_end;
    Intake intake = new Intake();
    public Servo servo = null;
    private boolean servoLast = false;
    private ElapsedTime servoTimer = new ElapsedTime();
    public TouchSystem slide = new TouchSystem();

    public void init(LinearOpMode linearOpMode) {
        //tele.optical = true;
        this.servo = (Servo) linearOpMode.hardwareMap.get(Servo.class, "frow");
        DigitalChannel digitalChannel = (DigitalChannel) linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        this.digitalTouch_end = digitalChannel;
        digitalChannel.setMode(DigitalChannel.Mode.INPUT);
        this.intake.init(linearOpMode);
        slide.init(linearOpMode);
    }
    public void setServoPosition(boolean servo2) {
        if (optical) {
            if (this.intake.isFreightDetected()) {
                this.blockTimer.reset();
            }
            boolean servo3 = servo2 && !this.digitalTouch_end.getState();
            if (servo3 && !this.servoLast && this.blockTimer.milliseconds() < 300.0d) {
                this.servoTimer.reset();
            }
            if (!servo3 || this.blockTimer.milliseconds() >= 300.0d) {
                this.servo.setPosition(position1);
            } else {
                this.servo.setPosition(Range.clip(position1 + (this.servoTimer.seconds() * (Math.abs(position2 - position1) / servoSpeedSeconds)), position1, position2));
            }
            this.servoLast = servo3;
        }
        else{
            if (servo2){
            this.servo.setPosition(position1);
            } else {
                this.servo.setPosition(Range.clip(position1 + (this.servoTimer.seconds() * (Math.abs(position2 - position1) / servoSpeedSeconds)), position1, position2));
            }
        }
    }
}
