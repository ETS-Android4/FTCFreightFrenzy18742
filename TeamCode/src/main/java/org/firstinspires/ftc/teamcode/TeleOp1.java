package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOp1 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final MotorBase motorBase = new MotorBase();
    private final DuckSpinner duckSpinner = new DuckSpinner();
    private final Lift lift = new Lift();
    private final Intake intake = new Intake();
    public Frow frow = new Frow();

    public boolean servo = false;
    private final ButtonSwitch forwardDuckSpinnerSwitch = new ButtonSwitch();
    private final ButtonSwitch reverseDuckSpinnerSwitch = new ButtonSwitch();
    private final ButtonSwitch liftSwitch = new ButtonSwitch();
    private final ButtonSwitch servoSwitch = new ButtonSwitch();
    private final ButtonSwitch intakeSwitch = new ButtonSwitch();
    private boolean intake_bool = false;
    //private boolean duckForward = forwardDuckSpinnerSwitch.updateSwitchState(gamepad1.square);
    //private boolean duckReverse = reverseDuckSpinnerSwitch.updateSwitchState(gamepad1.triangle);
    double duckPower = 1;
    double intake_counter = 1;
    public int lift_direction = 1;


    @Override
    public void runOpMode() {
        motorBase.init(this);
        duckSpinner.init(this);
        lift.init(this);
        frow.init(this);
        intake.init(this);

        waitForStart();
        while (opModeIsActive()) {
            intake_bool = intakeSwitch.updateSwitchState(gamepad1.circle);
            // Motor base
            double turn = gamepad1.left_stick_x;
            double drive = gamepad1.right_trigger - gamepad1.left_trigger;
            double side = gamepad1.right_stick_x;
            //motorBase.move(drive, side, turn);

            // Button functions
            lift_servo();
            //duck();
            intake_motor();
            lift();

        }
    }

        public void lift_servo(){
            servo = servoSwitch.updateSwitchState(gamepad1.circle);
            frow.setServoPosition(servo);
        }

        /*public void duck(){
            if (duckForward) duckSpinner.setMotorPower(duckPower);
            else if (duckReverse) duckSpinner.setMotorPower(-duckPower);
            else duckSpinner.setMotorPower(0);
        }*/

        public void lift(){
            if (liftSwitch.updateSwitchState(gamepad1.cross)) {
                lift.setMotorTarget(lift_direction);
                lift_direction = -lift_direction;
            }
        }

        public void intake_motor(){
            if(intake_bool){
                if(intake_counter % 2 == 0){intake.IntakeMotor(1);}
                else{intake.IntakeMotor(0);}
                intake_counter++;
            }
        }
}



