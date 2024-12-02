package org.firstinspires.ftc.teamcode.AllCodes;//package org.firstinspires.ftc.teamcode.AllCodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//
//
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SampleDropSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SamplePickSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SpecimenDropSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SpecimenPickSeq;
//import org.firstinspires.ftc.teamcode.Vision.GripperOrientPipeline;
//import org.firstinspires.ftc.teamcode.hardware.Globals;
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
//import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp
//@Config
//public class UPDATED_INTO extends LinearOpMode {
//
//
//    // TODO ================================================== SUBSYSTEMS ===========================================================================
//    private OuttakeSubsystem outtake;
//    private ElevatorSubsytem elevator;
//
//    // TODO ================================================== DRIVE ===============================================================================
//    private MecanumDrive drive;
//    private double botHeading;
//
//    // TODO ================================================== INSTANCES ============================================================================
//    private final RobotHardware robot = RobotHardware.getInstance();
//
//    // TODO ================================================= VARIABLES =============================================================================
//    public static Boolean HighBasket = true;
//    public boolean is90Degree = false;
//    public boolean isBasket = false;
//    public boolean isGripSample = false;
//    public boolean isDropSpecimen = false;
//    public double multiplier = 0;
//
//    // Todo ================================================== COLOR SENSOR ==================================================================
//    public int redThreshold=350;
//    public int BlueThreshold=420;
//    public int ThresholdDistance=17;
//    public boolean isPickSpecimen = false;
//    public boolean isPickSample = true;
//
//    // TODO  ================================================== Camera Integration =============================================================
//    GripperOrientPipeline pipeline = new GripperOrientPipeline();
//    public int CAMERA_HEIGHT = 720;
//    public int CAMERA_WIDTH = 1280;
//    public static List<Action> runningActions = new ArrayList<>();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        outtake = new OuttakeSubsystem(robot);
//        elevator = new ElevatorSubsytem(robot);
////        initCamera();
//
//        while (opModeInInit()) {
//            new InitSeq(outtake, elevator);
//
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()){
//            runningActions = updateAction();
//            botHeading = drive.pose.heading.toDouble();
//
//            // TODO ============================== Sample Drop ======================================================================
//
//            if(gamepad1.a && isBasket){
//                if(HighBasket){
//                    new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHBASKET, 0);
//                }
//                else {
//                    new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.LOWBASKET, 0);
//                }
//            }
//
//            if(gamepad2.a){ // Operator Controls
//                HighBasket = true;
//            }
//
//
//            else if(gamepad2.b){ // Operator Controls
//                HighBasket = false;
//            }
//
//            if(gamepad1.y){ // Dropped
//                runningActions.add(
//                        new SequentialAction(
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen)),
//                                new SleepAction(0.6),
//                                InitSeq.InitSeqAction(outtake, elevator)
//                        )
//                );
//                isBasket = false;
//                isPickSample = true;
//                isGripSample = false;
//            }
//
//            // TODO ============================== Sample Pick ======================================================================
//            if(gamepad1.right_bumper && isPickSample)
//            { // DRIVER CONTROL
//                // HIGH BASKET
//                runningActions.add(SamplePickSeq.SamplePickSeqAction(outtake,elevator));
//                isBasket = true;
//                isPickSpecimen = false;
//                isGripSample = true;
//            }
//
//            if(gamepad1.x && isGripSample)
//            { // Sample Picked
//                runningActions.add(
//                        new SequentialAction(
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen)),
//                                new InstantAction(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePrePick)),
//                                new SleepAction(0.5),
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
//                                new SleepAction(0.6),
//                                SampleDropSeq.SampleDropSeqAction(outtake)
//                        )
//                );
//
//            }
//
//            // TODO ============================= Specimen Drop ======================================================================
//
//            if(gamepad1.b && isDropSpecimen){
//
//                runningActions.add(
//                        new SequentialAction(
//                                new InstantAction(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0)),
//                                new SleepAction(0.6),
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                        )
//                );
//            }
//
//
//            // TODO ============================= Specimen Pick ======================================================================
//            if(gamepad1.right_trigger>0){
////                new SpecimenPickSeq(outtake, elevator);
//                runningActions.add(SpecimenPickSeq.SpecimenPickSeqAction(outtake,elevator));
//                isPickSpecimen = true;
//                isPickSample = false;
//                isDropSpecimen = true;
//            }
//
//            if(((robot.SensorColor1.red()>=redThreshold || robot.SensorColor1.blue()>=BlueThreshold)
//                    && robot.SensorColor1.getDistance(DistanceUnit.MM)<ThresholdDistance) && isPickSpecimen){
//                runningActions.add(
//                            new SequentialAction(
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
//                                new InstantAction(()->new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENPICKED, 0)),
//                                new SleepAction(0.5),
//                                SpecimenDropSeq.SpecimenDropSeqAction(outtake, elevator)
//                        )
//                );
//                isPickSpecimen = false;
//            }
//
//            if(gamepad2.left_bumper && isPickSpecimen){ // Manual Specimen Graping
//                runningActions.add(
//                        new SequentialAction(
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
//                                new SleepAction(0.5),
//                                SpecimenDropSeq.SpecimenDropSeqAction(outtake,elevator),
//                                new InstantAction(()->  new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENPICKED, 0))
//                        )
//                );
//                isPickSpecimen = false;
//            }
//
//            // TODO ===================================== Wrist 90 ===============================================================
//            if(gamepad2.x){
//                runningActions.add(
//                        new SequentialAction(
//                                new InstantAction(()-> robot.Wrist.setPosition(Globals.Wrist90)),
//                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                        )
//                );
//            }
//
//            //TODO ============================= Rack Extension ========================================================================
//            if(gamepad2.dpad_up){
//                outtake.setRackPos(Globals.rackExtensionMaxPos);
//            }
//            if(gamepad2.dpad_down){
//                outtake.setRackPos(Globals.rackExtensionMidPos);
//            }
//
//
//            // TODO ============================== Field Oriented   ====================================================================
//
//            DriveFieldCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x /2 , botHeading);
//            if(gamepad1.back){
//                drive.lazyImu.get().resetYaw();
//            }
//
//            if (gamepad1.left_trigger>0){
//                multiplier = 1.4  - gamepad1.left_trigger;
//                DriveFieldCentric(-gamepad1.left_stick_x*multiplier, -gamepad1.left_stick_y*multiplier, gamepad1.right_stick_x*multiplier , botHeading);
//            }
//
//            // Todo ==================================== Robot Oriented ======================================================================
////            drive.setDrivePowers(
////                    new PoseVelocity2d(
////                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x / 2)
////            );
////
////            if (gamepad1.left_trigger>0.3){
////                drive.setDrivePowers(
////                        new PoseVelocity2d(
////                                new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), -gamepad1.right_stick_x / 3)
////                );
////            }
//
//            telemetry.addData("Rack : ", robot.RackExtend.getPosition());
//            telemetry.addData("Gripper : ", robot.Gripper.getPosition());
//            telemetry.addData("Shoulder : ", robot.Shoulder.getPosition());
//            telemetry.addData("Arm : ", robot.Arm.getPosition());
//            telemetry.addData("Wrist Pos : ", robot.Wrist.getPosition());
//
//            telemetry.addData("leftCurrent : ",robot.LeftElevator.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("rightCurrent : ",robot.RightElevator.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Left Elevator POS : ",robot.LeftElevator.getCurrentPosition());
//            telemetry.addData("Right Elevator Pos : ",robot.RightElevator.getCurrentPosition());
//            telemetry.addData("Heading Degree : ", Math.toDegrees(drive.pose.heading.toDouble()));
//
//            telemetry.addData("Left Elevator Velocity : ",robot.LeftElevator.getVelocity());
//            telemetry.addData("Right Elevator Velocity : ",robot.RightElevator.getVelocity());
//
//            telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
//
//
//            telemetry.addData("Red Value", robot.SensorColor1.red());
//            telemetry.addData("Blue Value", robot.SensorColor1.blue());
//            telemetry.addData("Green Value", robot.SensorColor1.green());
//
//            telemetry.addData("Distance Value", robot.SensorColor1.getDistance(DistanceUnit.MM));
//            telemetry.addData("Gripper Angle", pipeline.getAngle());
//
//            telemetry.update();
//            drive.updatePoseEstimate();
//        }
//    }
//
//    public void DriveFieldCentric(double x, double y, double rx, double botHeading){
//
//        double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
//        double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading));
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//
//
//        double frontLeftPower = (rotY - rotX + rx) / denominator;
//        double backLeftPower = (rotY + rotX + rx) / denominator;
//        double frontRightPower = (rotY + rotX - rx) / denominator;
//        double backRightPower = (rotY - rotX - rx) / denominator;
//
//        drive.leftFront.setPower(frontLeftPower);
//        drive.leftBack.setPower(backLeftPower);
//        drive.rightBack.setPower(backRightPower);
//        drive.rightFront.setPower(frontRightPower);
//
//    }
//    private void initCamera(){
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        controlHubCam.setPipeline(pipeline);
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 60);
//    }
//
//    public void SetWrist(int pos){
//        if(pos == 90){
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new InstantAction(()-> robot.Wrist.setPosition(Globals.Wrist90)),
//                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                    )
//            );
//        }
//
//        if(pos == 0){
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new InstantAction(()-> robot.Wrist.setPosition(Globals.wristSamplePick)),
//                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                    )
//            );
//        }
//
//        if(pos == 45){
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new InstantAction(()-> robot.Wrist.setPosition(Globals.WristLeftDiagonal)),
//                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                    )
//            );
//        }
//
//        if(pos == 135){
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new InstantAction(()-> robot.Wrist.setPosition(Globals.WristRightDiagonal)),
//                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                    )
//            );
//        }
//
//    }
//
//    public static List<Action> updateAction(){
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> newActions = new ArrayList<>();
//        List<Action> RemovableActions = new ArrayList<>();
//
//        for (Action action : runningActions) {
////            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
////        runningActions.removeAll(RemovableActions);
//        return newActions;
//    }
//}
//
//// TODO ========================================================== DISCARDED PART =================================================================================
//
//
//// TODO ============================== Robot Oriented   ====================================================================
//
//
//
////            drive.setDrivePowers(
////                    new PoseVelocity2d(
////                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x / 2)
////            );
//
////            if (gamepad1.left_trigger>0.3){
////                drive.setDrivePowers(
////                        new PoseVelocity2d(
////                                new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), -gamepad1.right_stick_x / 3)
////                );
////            }
//
//
//
//// Operator
////            if((gamepad2.right_stick_x>0 || gamepad2.right_stick_x <0) && isPickSample ){
////                SetWrist(90);
////            }
////
////            if((gamepad2.right_stick_y>0 || gamepad2.right_stick_y <0) && isPickSample){
////                SetWrist(0);
////            }
////
////            if((gamepad2.right_stick_x>0 && gamepad2.right_stick_y>0) && isPickSample){
////                SetWrist(45);
////            }
////
////            if((gamepad2.right_stick_x<0 && gamepad2.right_stick_y>0) && isPickSample){
////                SetWrist(135);
////            }
//
//
////            if(gamepad1.left_bumper && isBasket){ // DRIVER CONTROL
////                // HIGH BASKET
////                new SampleDropSeq(outtake);
////
////                isBasket = false;
////                isPickSample = false;
////            }
