package org.firstinspires.ftc.teamcode.AllCodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.Sequence.SampleDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SamplePickSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenPickSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

@TeleOp()
@Config
public class Into_The_Deep extends LinearOpMode {


    // TODO ================================================== SUBSYSTEMS ===========================================================================
    private OuttakeSubsystem outtake;
    private ElevatorSubsytem elevator;


    // TODO ================================================== DRIVE ===============================================================================
    private MecanumDrive drive;
    private double botHeading;

    // TODO ================================================== INSTANCES ============================================================================
    private final RobotHardware robot = RobotHardware.getInstance();

    // TODO ================================================= VARIABLES =============================================================================
    public static Boolean HighBasket = true;

        public int ThresholdColor=1000;
    public int ThresholdDistance=15;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        outtake = new OuttakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);

        while (opModeInInit()) {
            // Do Nothing
            new InitSeq(outtake, elevator, ElevatorSubsytem.ElevateState.HOME);
        }

        waitForStart();

        while (opModeIsActive()){

            double botHeading = drive.pose.heading.toDouble();


            // TODO ============================== Sample Drop ======================================================================
            if(gamepad1.left_bumper){ // DRIVER CONTROL
                // HIGH BASKET

                new SampleDropSeq(outtake, elevator, HighBasket);
            }


            if(gamepad2.a){ // Operator Controls
                HighBasket = true;
            }
            else if(gamepad2.b){ // Operator Controls
                HighBasket = false;
            }

            if(gamepad1.y){ // Dropped
                robot.Gripper.setPosition(Globals.gripperOpen);
            }

            // TODO ============================== Sample Pick ======================================================================
            if(gamepad1.right_bumper){ // DRIVER CONTROL
                // HIGH BASKET
                new SamplePickSeq(outtake, elevator);
            }

            if(gamepad1.x){ // Sample Picked
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen)),
                                new InstantAction(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePrePick)),
                                new SleepAction(0.5),
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
                                new SleepAction(0.1),
                                new InstantAction(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePick1))
                        )
                );
            }

            // TODO ============================= Specimen Drop ======================================================================
            if(gamepad1.left_trigger>0){
                new SpecimenDropSeq(outtake, elevator);
            }

            if(gamepad1.b){
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0)),
                                new SleepAction(0.5),
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                        )
                );
            }


            // TODO ============================= Specimen Pick ======================================================================
            if(gamepad1.right_trigger>0){
                new SpecimenPickSeq(outtake, elevator);
            }


            if(((robot.SensorColor1.red()>=ThresholdColor || robot.SensorColor1.blue()>=ThresholdColor || robot.SensorColor1.green()>=ThresholdColor ) && robot.SensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)
                && ((robot.SensorColor1.red()>=ThresholdColor || robot.SensorColor1.blue()>=ThresholdColor || robot.SensorColor1.green()>=ThresholdColor) && robot.SensorColor1.getDistance(DistanceUnit.MM)<=ThresholdDistance)){
                telemetry.addLine("Got You");
        }

            if(gamepad1.a){
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
                                new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENPICKED, 0))
                        )
                );
            }

            // TODO ===================================== Wrist 90 ===============================================================
            if(gamepad2.x){
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(()-> robot.Wrist.setPosition(Globals.Wrist90))
                        )
                );
            }



            // TODO ============================== Field Oriented   ====================================================================

//            DriveFieldCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x /2 , botHeading);
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), -gamepad1.right_stick_x / 4)
            );

//            telemetry.addData("Rack : ", robot.RackExtend.getPosition());
            telemetry.addData("Gripper : ", robot.Gripper.getPosition());
            telemetry.addData("Shoulder : ", robot.Shoulder.getPosition());
            telemetry.addData("Arm : ", robot.Arm.getPosition());
            telemetry.addData("Wrist Pos : ", robot.Wrist.getPosition());

            telemetry.addData("leftCurrent : ",robot.LeftElevator.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightCurrent : ",robot.RightElevator.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Elevator POS : ",robot.LeftElevator.getCurrentPosition());
            telemetry.addData("Right Elevator Pos : ",robot.RightElevator.getCurrentPosition());
            telemetry.addData("Heading Degree : ", Math.toDegrees(drive.pose.heading.toDouble()));

            telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));


            telemetry.addData("Red Value", robot.SensorColor1.red());
            telemetry.addData("Blue Value", robot.SensorColor1.blue());
            telemetry.addData("Green Value", robot.SensorColor1.green());

            telemetry.addData("Distance Value", robot.SensorColor1.getDistance(DistanceUnit.MM));



            telemetry.update();
            drive.updatePoseEstimate();
        }
    }

    public void DriveFieldCentric(double x, double y, double rx, double botHeading){

        double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
        double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading));

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);


        double frontLeftPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
        drive.rightFront.setPower(frontRightPower);

    }
}

// TODO ========================================================== DISCARDED PART =================================================================================


// TODO ============================== Robot Oriented   ====================================================================


//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), gamepad1.right_stick_x / 4)
//            );
