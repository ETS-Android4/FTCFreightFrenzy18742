package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Activities.Bucket;
import org.firstinspires.ftc.teamcode.Sensors.ButtonSwitch;
import org.firstinspires.ftc.teamcode.Activities.Intake;
import org.firstinspires.ftc.teamcode.Activities.MotorBase;
import org.firstinspires.ftc.teamcode.Sensors.RobotPosition;
import org.firstinspires.ftc.teamcode.Activities.Slide;

@TeleOp
public class TeleOpSticks extends LinearOpMode {

    private final MotorBase motorBase = new MotorBase();
    private final Slide slide = new Slide();
    private final Intake intake = new Intake();
    public Bucket bucket = new Bucket();
    public final RobotPosition position = new RobotPosition();

    public boolean intakeBlock = false;
    public boolean liftBlock = false;
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
            double turn = gamepad1.left_stick_x;
            double drive = gamepad1.right_trigger - gamepad1.left_trigger;
            double side = gamepad1.right_stick_x;
            motorBase.move(drive*speed, side*speed, turn*speed);
            // Button functions
            bucketFunction();
            extraLift();
            slideFunction();
            intakeFunction();
            speedFunction();
            positionFunction();
            //speed = Math.abs(gamepad1.touchpad_finger_1_x);
        }
    }

    public void positionFunction(){
        telemetry.addData("X:", position.position_x());
        telemetry.addData("Y:", position.position_y());
        telemetry.addData("ANGLE(degrees):", position.position_angle());
        telemetry.update();
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
        if(intake.intakeMotor.getPower() < 0.1){ intakeBlock = false; }
        else {intakeBlock = true;}
    }

    public void extraLift() {
        if (gamepad1.dpad_down && gamepad1.dpad_left) {
            extra = true;
        } else {
            extra = false;
        }
    }

    public void speedFunction(){
        if(speedSwitch.updateSwitchState(gamepad1.dpad_up) || intakeBlock){ speed = 0.3; }
        else{ speed = 1.0; }
    }
}



