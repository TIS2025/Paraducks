package org.firstinspires.ftc.teamcode.AllCodes;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@
        TeleOp(name = "TwoMotorControl", group = "Linear Opmode")
public class TwoMotorControl extends LinearOpMode {

    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private Servo s1 = null;
//5100
    //public static int leftInc=0;
    //public static int rightInc=0;
    public static int slider = -5000 ;
//790
    //3700
    public static int fpos=786;
    public static int fdownpos=2800;



    public static int hang = 100;

    @Override
    public void runOpMode() {

        // Initialize hardware
        motorLeft = hardwareMap.get(DcMotor.class, "M1");
        motorRight = hardwareMap.get(DcMotor.class, "M2");
        s1 = hardwareMap.get(Servo.class, "s1");

        // Reset encoders and set to RUN_TO_POSITION mode
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        s1.setPosition(0.5);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//588
        // Example usage: move both motors to target position with power
        extendTo(0.5, 0);

        while (opModeIsActive()) {

            if(gamepad1.left_bumper){
                motorLeft.setTargetPosition(fpos);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeft.setPower(1);
            }
            if(gamepad1.a){
                motorLeft.setTargetPosition(motorLeft.getCurrentPosition()+50);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeft.setPower(1);
            } else if (gamepad1.b) {
                motorLeft.setTargetPosition(motorLeft.getCurrentPosition()-50);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeft.setPower(1);
            }
////////////////////////////////////////////////////////////////////////////////////////
            if(gamepad1.right_bumper){
                motorRight.setTargetPosition(slider);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setPower(1);

                motorLeft.setTargetPosition(fdownpos);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLeft.setPower(1);

            }

            if(gamepad1.left_trigger>0.5){
                motorRight.setTargetPosition(hang);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setPower(1);



            }

            if(gamepad1.x){
                motorRight.setTargetPosition(motorRight.getCurrentPosition()+50);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setPower(1);
            } else if (gamepad1.y) {
                motorRight.setTargetPosition(motorRight.getCurrentPosition()-50);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setPower(1);
            }
            telemetry.addData("Left Position", motorLeft.getCurrentPosition());
            telemetry.addData("Right Position", motorRight.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Method to extend two motors to a given target position with a specific power
     * @param pow Power to set the motors (-1 to 1)
     * @param targetPos Target position (in encoder counts) to move motors
     */


    public void extendTo(double pow, int targetPos) {
        // Set the target position for both motors
        motorLeft.setTargetPosition(targetPos);
        motorRight.setTargetPosition(targetPos);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power to control the motors
        motorLeft.setPower(pow);
        motorRight.setPower(pow);

    }

    public void extendLeftTo(double pow, int targetPos) {
        // Set the target position for the left motor
        motorLeft.setTargetPosition(targetPos);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set the power to control the left motor
        motorLeft.setPower(pow);


    }

    /**
     * Method to extend the right motor to a given target position with a specific power
     * @param pow Power to set the right motor (-1 to 1)
     * @param targetPos Target position (in encoder counts) to move the right motor
     */
    public void extendRightTo(double pow, int targetPos) {
        // Set the target position for the right motor
        motorRight.setTargetPosition(targetPos);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power to control the right motor
        motorRight.setPower(pow);

    }
}

