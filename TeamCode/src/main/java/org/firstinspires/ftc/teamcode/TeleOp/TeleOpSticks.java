package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Activities.AutoPlus;
import org.firstinspires.ftc.teamcode.Activities.Bucket;
import org.firstinspires.ftc.teamcode.OpenCV.ArucoDetect;
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
    public final AutoPlus autoPlus = new AutoPlus();
    public final ArucoDetect arucoDetect = new ArucoDetect();

    private ElapsedTime timerAuto = new ElapsedTime();
    public boolean intakeBlock = false;
    public boolean liftBlock = false;
    public boolean extra = false;
    private final ButtonSwitch liftSwitch = new ButtonSwitch();
    private final ButtonSwitch servoSwitch = new ButtonSwitch();
    private final ButtonSwitch intakeSwitch = new ButtonSwitch();
    private final ButtonSwitch speedSwitch = new ButtonSwitch();
    private double speed = 1.0;
    private boolean slideButton;
    private double a = 0.1, i = 0;
    public boolean ledRange = true;

    @Override
    public void runOpMode() {
        arucoDetect.initialize(this);
        motorBase.init(this);
        slide.init(this);
        bucket.init(this);
        intake.init(this);
        autoPlus.init(this);
        //position.init(this);

        waitForStart();
        while (opModeIsActive()) {
            // Motor base
            double turn = gamepad1.left_stick_x;
            double drive = gamepad1.right_trigger - gamepad1.left_trigger;
            double side = gamepad1.right_stick_x;
            motorBase.move(drive * speed, side * speed, turn * speed);
            // Button functions
            bucketFunction();
            // extraLift();
            slideButton = gamepad1.cross;
            slideFunction();
            intakeFunction();

            /*telemetry.addData("start", !slide.touchSystem.digitalTouch_start.getState());
            telemetry.addData("end", !slide.touchSystem.digitalTouch_end.getState());
            telemetry.addData("LED", 1.0/60.0*i);
            telemetry.addData("Forward distance", motorBase.getForwardDistance());
            telemetry.addData("Side distance", motorBase.getSideDistance());
            telemetry.addData("1st", motorBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("2nd", motorBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("3rd", motorBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);*/
            telemetry.addData("ForcePosition",arucoDetect.forceGetPosition());
            telemetry.addData("TimePosition", arucoDetect.timePosition);
            telemetry.update();

            if(ledRange){i+=a; a+=0.1;}
            else{i-=a; a+=0.1;}
            if(1.0/60.0*i > 1.0){ledRange = !ledRange; a = 0.2; i = 60;}
            if(1.0/60.0*i < 0.0){ledRange = !ledRange; a = 0.2; i = 0;}
            autoPlus.ledPlus.setPower(1.0/60.0*i);
            // slideAuto();
            // speedFunction();
            // positionFunction()
            // speed = Math.abs(gamepad1.touchpad_finger_1_x);
        }
    }

    /*public void positionFunction(){
        telemetry.addData("X:", position.positionX());
        telemetry.addData("Y:", position.positionY());
        telemetry.addData("ANGLE(degrees):", position.positionAngleDegrees());
        telemetry.update();
    }*/

    public void bucketFunction() {
        bucket.setServoPosition(servoSwitch.updateSwitchState(gamepad1.circle ));
    }

    public void slideFunction() {
        if (liftSwitch.updateSwitchState(slideButton))
            slide.setMotorTarget(1, extra);
        else slide.setMotorTarget(-1, extra);
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
        if (intake.intakeMotor.getPower() < 0.1) {
            intakeBlock = false;
        } else {
            intakeBlock = true;
        }
    }

    public void extraLift() {
        if (gamepad1.dpad_down && gamepad1.dpad_left) {
            extra = true;
        } else {
            extra = false;
        }
    }

    public void speedFunction() {
        if (speedSwitch.updateSwitchState(gamepad1.dpad_up) || intakeBlock) {
            speed = 0.3;
        } else {
            speed = 1.0;
        }
    }

    public void slideAuto() {
        if (intake.isFreightDetected()) {
            timerAuto.reset();
        }
        if ((timerAuto.milliseconds() > 1000) && (timerAuto.milliseconds() < 1500)) {
            slide.setMotorTarget(1, false);
        }
    }

    public void ledRise(boolean d){
        if(d){ while(i<61){ autoPlus.ledPlus.setPower(1.0/60.0*i); i+=a; a++; sleep(100);} }
        else{ while(i>0){ autoPlus.ledPlus.setPower(1.0/60.0*i); i-=a; a++; sleep(100);} }
    }

}



