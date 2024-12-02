package org.firstinspires.ftc.teamcode.AllCodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "GetValues")
@Config
public class GetValues extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    // TODO ============================================= OUTTAKE/INTAKE PART =============================================================
    public double shoulderpos=0.5;
    public double rackPos=0.5;
    public double gripperPos=0.5;
    public double armPos=0.5;
    public static double wristPos=0.5;
    public static double power=0.8;

    // TODO =============================================== ELEVATOR PART ===========================================================
    public static int leftExtensionPos=0;
    public static int rightExtensionPos=0;


    // TODO ================================================ HANGER PART ============================================================
    public static double hanger1Pos = 0;
    public static double hanger2Pos = 0;
    public static int TiltMotorPos=0;




    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        while (opModeInInit()){
            setServoShoulder(shoulderpos); // Both Shoulder
            robot.Gripper.setPosition(gripperPos); // Left Gripper
            robot.RackExtend.setPosition(rackPos); // Rotate
            robot.Arm.setPosition(armPos);
            robot.Wrist.setPosition(wristPos);
            setRackPos(rackPos);
        }


        waitForStart();


        while(opModeIsActive()){

            // TODO===================================================== RackExtend Servo (1,a,b) =================================================================
            if(gamepad1.a){
                rackPos+=0.001;
                robot.RackExtend.setPosition(rackPos);
            }
            else if (gamepad1.b) {
                rackPos-=0.001;
                robot.RackExtend.setPosition(rackPos);
            }
            // TODO===================================================== Shoulder Servo(1,dpad-left,right) =================================================================
            else if (gamepad1.back) {
                shoulderpos += 0.001;
                setServoShoulder(shoulderpos);
            }

            else if (gamepad1.start) {
                shoulderpos -= 0.001;
                setServoShoulder(shoulderpos);
            }

            // TODO===================================================== Wrist Servo =================================================================
            else if (gamepad1.left_bumper) {
                wristPos -= 0.001;
                robot.Wrist.setPosition(wristPos);
            }
            else if (gamepad1.right_bumper) {
                wristPos += 0.001;
                robot.Wrist.setPosition(wristPos);
            }

            // TODO===================================================== Grip Servo =================================================================
            if (gamepad1.x) {
                gripperPos+=0.001;
                robot.Gripper.setPosition(gripperPos);
            }
            else if (gamepad1.y) {
                gripperPos-=0.001;
                robot.Gripper.setPosition(gripperPos);
            }
            // TODO===================================================== ARM Servo =================================================================
            else if (gamepad1.dpad_up) {
                armPos+=0.001;
                robot.Arm.setPosition(armPos);

            }else  if (gamepad1.dpad_down) {
                armPos-=0.001;
                robot.Arm.setPosition(armPos);
            }


            // TODO===================================================== Elevator Motors  =================================================================
            else if (gamepad1.left_trigger>0) {
                lifterINC();
            }
            else if (gamepad1.right_trigger>0) {
                lifterDEC();
            }


            // TODO===================================================== Hanger1 Servo =================================================================
            else if (gamepad1.left_bumper) {
                hanger1Pos -= 0.001;
                robot.Hanger1.setPosition(hanger1Pos);
            }
            else if (gamepad1.right_bumper) {
                hanger1Pos += 0.001;
                robot.Hanger1.setPosition(hanger1Pos);
            }

            // TODO===================================================== Hanger2 Servo =================================================================
            if (gamepad2.x) {
                hanger2Pos+=0.001;
                robot.Hanger2.setPosition(hanger2Pos);
            }
            else if (gamepad2.y) {
                hanger2Pos-=0.001;
                robot.Hanger2.setPosition(hanger2Pos);
            }

            // TODO===================================================== Tilt Motor  =================================================================
            else if (gamepad1.left_trigger>0) {
                TiltInc();
            }
            else if (gamepad1.right_trigger>0) {
                TiltDec();
            }


            // TODO ====================================================== RACK EXTENSION ==============================================
            else if(gamepad2.a){
                rackPos +=0.001;
                setRackPos(rackPos);
            }
            else if(gamepad2.b){
                rackPos -=0.001;
                setRackPos(rackPos);
            }


//            // TODO ============================================ Drive ===========================================================
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x)
            );

//            telemetry.addData("Rack : ", robot.RackExtend.getPosition());
            telemetry.addData("Gripper : ", robot.Gripper.getPosition());
            telemetry.addData("Shoulder : ", robot.Shoulder.getPosition());
            telemetry.addData("Arm : ", robot.Arm.getPosition());
            telemetry.addData("Wrist Pos : ", robot.Wrist.getPosition());
            telemetry.addData("Rack : ", robot.RackExtend.getPosition());

            telemetry.addData("Rack Extension Left Pos : ", robot.RackExtensionLeft.getPosition());
            telemetry.addData("Rack Extension Right Pos : ", robot.RackExtensionRight.getPosition());

            telemetry.addData("Hanger1 Pos : ", robot.Hanger1.getPosition());
            telemetry.addData("Hanger2 Pos : ", robot.Hanger2.getPosition());

            telemetry.addData("leftCurrent : ",robot.LeftElevator.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightCurrent : ",robot.RightElevator.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("TiltCurrent : ",robot.TiltMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Tilt POS : ",robot.TiltMotor.getCurrentPosition());
            telemetry.addData("Left Elevator POS : ",robot.LeftElevator.getCurrentPosition());
            telemetry.addData("Right Elevator Pos : ",robot.RightElevator.getCurrentPosition());
            telemetry.addData("Heading Degree : ", Math.toDegrees(drive.pose.heading.toDouble()));

            telemetry.update();
        }



    }

    public void setRackPos(double pos){

        robot.RackExtensionRight.setPosition(pos);
        robot.RackExtensionLeft.setPosition(1 - (pos-0.03));

    }



    public void setServoShoulder(double leftPos){
        robot.Shoulder.setPosition(leftPos);
    }



    public void lifterINC()
    {

        robot.RightElevator.setTargetPosition(robot.RightElevator.getCurrentPosition()-50);
        robot.RightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RightElevator.setPower(power);

        robot.LeftElevator.setTargetPosition((robot.LeftElevator.getCurrentPosition()+50));
        robot.LeftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LeftElevator.setPower(power);

    }

    public void TiltInc()
    {
        robot.TiltMotor.setTargetPosition((robot.TiltMotor.getCurrentPosition()+50));
        robot.TiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.TiltMotor.setPower(power);
    }

    public void TiltDec()
    {
        robot.TiltMotor.setTargetPosition((robot.TiltMotor.getCurrentPosition()-50));
        robot.TiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.TiltMotor.setPower(power);
    }
    public void lifterDEC(){

        robot.RightElevator.setTargetPosition(robot.RightElevator.getCurrentPosition()+50);
        robot.RightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RightElevator.setPower(power);

        robot.LeftElevator.setTargetPosition((robot.LeftElevator.getCurrentPosition()-50));
        robot.LeftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LeftElevator.setPower(power);

    }
}