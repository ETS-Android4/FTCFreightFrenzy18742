package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpTouchPad extends LinearOpMode {

    private final MotorBase motorBase = new MotorBase();
    private final Slide slide = new Slide();
    private final Intake intake = new Intake();
    public Bucket bucket = new Bucket();

    public boolean extra = false;
    private final ButtonSwitch liftSwitch = new ButtonSwitch();
    private final ButtonSwitch servoSwitch = new ButtonSwitch();
    private final ButtonSwitch intakeSwitch = new ButtonSwitch();
    private final ButtonSwitch speedSwitch = new ButtonSwitch();
    private double speed = 1.0;

    @Override
    public void runOpMode() {
        motorBase.init(this);
        slide.init(this);
        bucket.init(this);
        intake.init(this);

        waitForStart();
        while (opModeIsActive()) {
            // Motor base
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;
            double drive = gamepad1.touchpad_finger_1_y;
            double side = gamepad1.touchpad_finger_1_x;
            if(gamepad1.touchpad){ motorBase.move(drive*speed, side*speed, turn*speed); }
            else { motorBase.move(0, 0, 0); }
            // Button functions
            bucketFunction();
            extraLift();
            slideFunction();
            intakeFunction();
            speedFunction();

            telemetry.addLine("Endstop").addData("start", slide.digitalTouch_start.getState()).addData("end", slide.digitalTouch_end.getState());
            telemetry.update();

        }
    }

    public void bucketFunction() {
        bucket.setServoPosition(servoSwitch.updateSwitchState(gamepad1.circle));
    }

    public void slideFunction() {
        if (liftSwitch.updateSwitchState(gamepad1.cross)) slide.setMotorTarget(-1, extra);
        else slide.setMotorTarget(1, extra);
    }

    public void intakeFunction() {
        if (gamepad1.square)
            intake.IntakeMotor(-1);
        else {
            if (intakeSwitch.updateSwitchState(gamepad1.triangle))
                intake.IntakeMotor(1);
            else
                intake.IntakeMotor(0);
        }
    }

    public void extraLift() {
        if (gamepad1.dpad_down && gamepad1.dpad_left) {
            extra = true;
        } else {
            extra = false;
        }
    }

    public void speedFunction(){
        if(speedSwitch.updateSwitchState(gamepad1.dpad_up)){
            speed = 0.5;
        }
        else{
            speed = 1.0;
        }
    }
}



