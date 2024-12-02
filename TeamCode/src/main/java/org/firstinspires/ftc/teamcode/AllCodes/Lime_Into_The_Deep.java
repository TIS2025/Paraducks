package org.firstinspires.ftc.teamcode.AllCodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.Sequence.SampleDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SamplePickSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenPickSeq;
import org.firstinspires.ftc.teamcode.Vision.GripperOrientPipeline;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.KinematicSolver;
import org.firstinspires.ftc.teamcode.utils.PerspectiveSolver;
import org.firstinspires.ftc.teamcode.utils.Sample;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class Lime_Into_The_Deep extends LinearOpMode {


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
    public boolean is90Degree = false;
    public boolean isBasket = false;
    public boolean isGripSample = false;
    public boolean isDropSpecimen = false;
    public double multiplier = 0;
    public boolean isPickSpecimen = false;
    public boolean isPickSample = true;
    public boolean SamplePicked = false;
    public boolean flipped = false;

    public static List<Action> ftc = new ArrayList<>();

    // Todo ================================================== COLOR SENSOR ==================================================================
    public int redThreshold = 350;
    public int BlueThreshold = 420;
    public int ThresholdDistance = 17;


    // TODO  ================================================== Camera Integration =============================================================
    GripperOrientPipeline pipeline = new GripperOrientPipeline();

    // TODO ==================================================== Drive variables ==============================================================
    public ElapsedTime esp;
    public double strafe = 1, speed = 1, turn = 1;

    // TODO ==================================================== Lime Light Variables ===========================================================
//    Limelight3A limelight;

    public List<Sample>
            redSamples = new ArrayList<>();
    public List<Sample> blueSamples = new ArrayList<>();
    public List<Sample> yellowSamples = new ArrayList<>();
    //    Sample x = new Sample();
    int CAMERA_HEIGHT = 480;
    int CAMERA_WIDTH = 640;

    double w1 = 5.75;
    double w2 = 14.75;
    double cx1 = CAMERA_HEIGHT - 480;
    double cx2 = CAMERA_HEIGHT - 127;
    double cx3 = CAMERA_HEIGHT - 39;
    public static double angle = 0;
    double cam_offset = 7;
    double x_offset = 0;
    double y_offset = 0;
    Point obj_pos = new Point(0,0);
    Point[] prev_pos = {new Point(0,0),new Point(0,0),new Point(0,0)};
    boolean obj_orient_red = false;
    boolean obj_orient_yellow = false;
    boolean obj_orient_blue = false;

    PerspectiveSolver Psolver = new PerspectiveSolver(angle,x_offset,y_offset,cx1,cx2,cx3,0,10,20,
            0,0,0,0,0,0,w1,w2,CAMERA_HEIGHT,CAMERA_WIDTH);

    double pickup_pos;
    double pos;
    Boolean isBlueSample = true;
    Boolean isYellowSample = false;
    KinematicSolver solver = new KinematicSolver(2.5,-1.5,0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        outtake = new OuttakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);
        telemetry.setMsTransmissionInterval(100);
        robot.limelight.pipelineSwitch(2);

        while (opModeInInit()) {
            new InitSeq(outtake, elevator);
            robot.LED1GREEN.setState(true);
            robot.LED1RED.setState(true);

        }

        waitForStart();
        robot.limelight.start();

        while (opModeIsActive()) {

            redSamples.clear();
            blueSamples.clear();
            yellowSamples.clear();
            LLResult result = robot.limelight.getLatestResult();
            sample_filter(result);
            ftc = updateAction();
            botHeading = drive.pose.heading.toDouble();

            // TODO ============================== Sample Drop ======================================================================

            if (gamepad1.a && isBasket) {
                speed = 0.5;
                strafe = 0.5;
                turn = 0.5;
                if (HighBasket) {
                    new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHBASKET, 0);
                } else {
                    new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.LOWBASKET, 0);
                }
            }

            if (gamepad2.a) { // Operator Controls
                HighBasket = true;
            } else if (gamepad2.b) { // Operator Controls
                HighBasket = false;
            }

            if (gamepad1.y) { // Dropped
                ftc.add(
                        new SequentialAction(
                                new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperOpen)),
                                new SleepAction(0.6),
                                InitSeq.InitSeqAction(outtake, elevator)
                        )
                );
                isBasket = false;
                isPickSample = true;
                isGripSample = false;
                isPickSpecimen = false;
                speed = 1;
                strafe = 1;
                turn = 1;
                flipped = false;
            }

            // TODO ============================== Sample Pick ======================================================================
            if (gamepad1.right_bumper && isPickSample) { // DRIVER CONTROL
                // HIGH BASKET
                if(isBlueSample){
                    ftc.add(
                            new SequentialAction(
                                    SamplePickSeq.SamplePickSeqAction(outtake, elevator),
                                    new InstantAction(()->{
                                        if(!blueSamples.isEmpty()){
                                            pickup_pos = solver.getExt(blueSamples.get(0).field_pos);
                                            obj_orient_blue = blueSamples.get(0).orientation;
                                            pos = obj_orient_blue?0.4339:0.715;
                                        }
                                    }),
                                    new InstantAction(()-> outtake.setRackPosInches(pickup_pos)),
                                    new InstantAction(()-> outtake.WristServo(pos))
                            )
                    );
                }

                if(isYellowSample){
                    ftc.add(
                            new SequentialAction(
                                    SamplePickSeq.SamplePickSeqAction(outtake, elevator),
                                    new InstantAction(()->{
                                        if(!yellowSamples.isEmpty()){
                                            pickup_pos = solver.getExt(yellowSamples.get(0).field_pos);
                                            obj_orient_yellow = yellowSamples.get(0).orientation;
                                            pos = obj_orient_yellow?0.4339:0.715;
                                        }
                                    }),
                                    new InstantAction(()-> outtake.setRackPosInches(pickup_pos)),
                                    new InstantAction(()-> outtake.WristServo(pos))
                            )
                    );
                }
                isPickSpecimen = false;
                isGripSample = true;
                speed = 0.5;
                strafe = 0.5;
                turn = 0.3;
            }

            if(gamepad1.x && isGripSample){ // Sample Picked
                ftc.add(
                        new SequentialAction(
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen)),
                                new InstantAction(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePrePick)),
                                new SleepAction(0.1),
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
                                new SleepAction(0.2),
                                new InstantAction(() -> {
                                    if (robot.SensorColor1.getDistance(DistanceUnit.MM)<ThresholdDistance) {
                                        Actions.runBlocking(
                                                new SequentialAction(
                                                        new InstantAction(() -> robot.Shoulder.setPosition(Globals.shoulderSamplePick))
                                                )
                                        );
                                        if (robot.SensorColor1.getDistance(DistanceUnit.MM)<ThresholdDistance) {
                                            Actions.runBlocking(
                                                    new SequentialAction(
                                                            new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                                                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                                                            new InstantAction(() -> SamplePicked = true),
//                                                            new InstantAction(() -> isGripSample = false),
                                                            new InstantAction(() -> speed = 1),
                                                            new InstantAction(() -> strafe = 1),
                                                            new InstantAction(() -> turn = 1),
                                                            new InstantAction(()-> robot.Arm.setPosition(0.2)),
                                                            new InstantAction(() -> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                                                            new InstantAction(() -> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.INIT)),
                                                            new InstantAction(() -> flipped = false)
                                                    )
                                            );
                                        } else {
                                            Actions.runBlocking(
                                                    new SequentialAction(
                                                            SamplePickSeq.SamplePickSeqAction(outtake, elevator)
                                                    )
                                            );
                                        }
                                    } else {
                                        Actions.runBlocking(
                                                new SequentialAction(
                                                        SamplePickSeq.SamplePickSeqAction(outtake, elevator)
                                                )
                                        );
                                    }
                                })
                        )
                );
            }

            // TODO ============================= Specimen Drop ======================================================================

            if (gamepad1.b && isDropSpecimen) {

                ftc.add(
                        new SequentialAction(
                                new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0)),
                                new SleepAction(0.6),
                                new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperOpen)),
                                new SleepAction(0.3),
                                new InstantAction(() -> new InitSeq(outtake, elevator))
                        )
                );
                isPickSample = true;
                isBasket = false;
                isGripSample = false;
            }


            // TODO ============================= Specimen Pick ======================================================================
            if (gamepad1.right_trigger > 0) {
                ftc.add(SpecimenPickSeq.SpecimenPickSeqAction(outtake, elevator));
                isPickSpecimen = true;
                isPickSample = false;
                isDropSpecimen = true;
                speed = 0.5;
                strafe = 0.5;
                turn = 0.3;
            }

            if (((robot.SensorColor1.red() >= redThreshold || robot.SensorColor1.blue() >= BlueThreshold)
                    && robot.SensorColor1.getDistance(DistanceUnit.MM) < ThresholdDistance) && isPickSpecimen) {
                ftc.add(
                        new SequentialAction(
                                new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperClose)),
                                new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENPICKED, 0)),
                                new SleepAction(0.5),
                                SpecimenDropSeq.SpecimenDropSeqAction(outtake, elevator)
                        )
                );
                isPickSpecimen = false;
                speed = 1;
                strafe = 1;
                turn = 1;
            }

            if (gamepad2.left_bumper && isPickSpecimen) { // Manual Specimen Graping
                Actions.runBlocking(
                        new SequentialAction(
                                new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperClose)),
                                new SleepAction(0.5),
                                SpecimenDropSeq.SpecimenDropSeqAction(outtake, elevator),
                                new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENPICKED, 0))
                        )
                );
                isPickSpecimen = false;
            }

            // TODO ===================================== Wrist 90 ===============================================================
            if (gamepad2.x && isGripSample) {
                if (flipped) {
                    ftc.add(
                            new SequentialAction(
                                    new InstantAction(() -> robot.Wrist.setPosition(Globals.wristInit)),
                                    new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperOpen)),
                                    new SleepAction(0.1),
                                    new InstantAction(() -> flipped = false)
                            )
                    );
                } else {
                    ftc.add(
                            new SequentialAction(
                                    new InstantAction(() -> robot.Wrist.setPosition(Globals.Wrist90)),
                                    new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperOpen)),
                                    new SleepAction(0.1),
                                    new InstantAction(() -> flipped = true)
                            )
                    );
                }
            }

            //TODO ============================= Rack Extension ========================================================================
            if (gamepad2.dpad_up && isGripSample) {
                outtake.setRackPos(Globals.rackExtensionMaxPos);
            }

            if (gamepad2.dpad_down && isGripSample) {
                outtake.setRackPos(Globals.rackExtensionMidPos);
            }


            // TODO ============================== Field Oriented   ====================================================================

            DriveFieldCentric(-gamepad1.left_stick_x * strafe, -gamepad1.left_stick_y * speed, gamepad1.right_stick_x * turn, botHeading);

            if (gamepad1.back) {
                drive.lazyImu.get().resetYaw();
                botHeading = drive.pose.heading.toDouble();
            }

            if (gamepad1.left_trigger > 0) {
                multiplier = 0.5;
                DriveFieldCentric(-gamepad1.left_stick_x * multiplier * strafe, -gamepad1.left_stick_y * multiplier * speed, gamepad1.right_stick_x * multiplier * turn, botHeading);
            } else {
                DriveFieldCentric(-gamepad1.left_stick_x * strafe, -gamepad1.left_stick_y * speed, gamepad1.right_stick_x * turn, botHeading);
            }
            // Todo ==================================== Robot Oriented ======================================================================
//            drive.setDrivePowers(
//                new PoseVelocity2d(
//                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x / 2)
//                    );
//
//            if (gamepad1.left_trigger>0.3){
//                drive.setDrivePowers(
//                        new PoseVelocity2d(
//                                new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), -gamepad1.right_stick_x / 3)
//                );
//            }
            if (gamepad2.y && SamplePicked) {
                ftc.add(
                        new SequentialAction(
                                SampleDropSeq.SampleDropSeqAction(outtake)
                        )
                );
                SamplePicked = false;
                isBasket = true;
            }

            // TODO ===================================================== Lime Light Integration =================================================
            if(gamepad2.right_bumper){
                isBlueSample = true;
                isYellowSample = false;
            }

            if(gamepad2.right_trigger>0){
                isYellowSample = true;
                isBlueSample = false;

            }

            if(!blueSamples.isEmpty()){ // Red On
                robot.LED1RED.setState(false);
                robot.LED2RED.setState(false);

                robot.LED1GREEN.setState(true);
                robot.LED2GREEN.setState(true);

            }
            if(!yellowSamples.isEmpty()){ // Green On

                robot.LED1GREEN.setState(false);
                robot.LED2GREEN.setState(false);


                robot.LED1RED.setState(true);
                robot.LED2RED.setState(true);
            }
            else {

                robot.LED1GREEN.setState(true);
                robot.LED2GREEN.setState(true);
                robot.LED2RED.setState(true);
                robot.LED1RED.setState(true);
            }

//            telemetry.addData("Rack : ", robot.RackExtend.getPosition());
//            telemetry.addData("Gripper : ", robot.Gripper.getPosition());
//            telemetry.addData("Shoulder : ", robot.Shoulder.getPosition());
//            telemetry.addData("Arm : ", robot.Arm.getPosition());
//            telemetry.addData("Wrist Pos : ", robot.Wrist.getPosition());
//
//            telemetry.addData("leftCurrent : ", robot.LeftElevator.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("rightCurrent : ", robot.RightElevator.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Left Elevator POS : ", robot.LeftElevator.getCurrentPosition());
//            telemetry.addData("Right Elevator Pos : ", robot.RightElevator.getCurrentPosition());
//            telemetry.addData("Heading Degree : ", Math.toDegrees(drive.pose.heading.toDouble()));
//
//            telemetry.addData("Left Elevator Velocity : ", robot.LeftElevator.getVelocity());
//            telemetry.addData("Right Elevator Velocity : ", robot.RightElevator.getVelocity());
//
//            telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
//
//
//            telemetry.addData("Red Value", robot.SensorColor1.red());
//            telemetry.addData("Blue Value", robot.SensorColor1.blue());
//            telemetry.addData(
//                    "Green Value", robot.SensorColor1.green());
//
//            telemetry.addData("Distance Value", robot.SensorColor1.getDistance(DistanceUnit.MM));
//            telemetry.addData("Gripper Angle", pipeline.getAngle());
            if(!redSamples.isEmpty()) {
                telemetry.addData("Red Samples",redSamples.size());
                telemetry.addData("Sample",redSamples.get(0).field_pos);
            }
            else telemetry.addData("Red Samples",0);
            if(!blueSamples.isEmpty())  {
                telemetry.addData("Blue Samples",blueSamples.size());
                telemetry.addData("Sample",blueSamples.get(0).field_pos);
            }
            else telemetry.addData("Blue Samples",0);
            if(!yellowSamples.isEmpty())  {
                telemetry.addData("Yellow Samples",yellowSamples.size());
                telemetry.addData("Sample",yellowSamples.get(0).field_pos);
            }
            else telemetry.addData("Yellow Samples",0);


            telemetry.addData("ext",pickup_pos);
            telemetry.addData("red_orient",obj_orient_red);
            telemetry.addData("blue_orient",obj_orient_blue);
            telemetry.addData("yellow_orient",obj_orient_yellow);

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
    private void initCamera(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(pipeline);
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 60);
    }

    public void SetWrist(int pos){
        if(pos == 90){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> robot.Wrist.setPosition(Globals.Wrist90)),
                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                    )
            );
        }

        if(pos == 0){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> robot.Wrist.setPosition(Globals.wristSamplePick)),
                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                    )
            );
        }

        if(pos == 45){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> robot.Wrist.setPosition(Globals.WristLeftDiagonal)),
                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                    )
            );
        }

        if(pos == 135){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> robot.Wrist.setPosition(Globals.WristRightDiagonal)),
                            new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                    )
            );
        }

    }

    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : ftc) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
//        runningActions.removeAll(RemovableActions);
        return newActions;
    }


    public void sample_filter(LLResult result){
        try{
            for (LLResultTypes.DetectorResult detector : result.getDetectorResults()) {
                List<Double> pt1 = detector.getTargetCorners().get(0);
                List<Double> pt2 = detector.getTargetCorners().get(1);
                List<Double> pt3 = detector.getTargetCorners().get(2);
                List<Double> pt4 = detector.getTargetCorners().get(3);

                double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
                double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
                double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
                double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

                double width = max_x - min_x;
                double height = max_y - min_y;

                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

                Point field_pos = Psolver.getX2Y2(new Point(cx, cy));
                boolean orientation = width/height>1.3    ;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();
                double offset = orientation?0.5:1.5;

                if (Math.abs(field_pos.y) <1.5 && field_pos.x < 12 && class_id == 0) {
                    field_pos.x+=offset;
                    blueSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                if (Math.abs(field_pos.y) <1.5 && field_pos.x < 12 && class_id == 1) {
                    field_pos.x+=offset;
                    redSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                if (Math.abs(field_pos.y) < 1.5 && field_pos.x < 12 && class_id == 2) {
                    field_pos.x+=offset;
                    yellowSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
            }
        }
        catch (Exception e){
            telemetry.addLine("Failed to input samples");
        }
    }



}


// TODO ========================================================== DISCARDED PART =================================================================================


// TODO ============================== Robot Oriented   ====================================================================



//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x / 2)
//            );

//            if (gamepad1.left_trigger>0.3){
//                drive.setDrivePowers(
//                        new PoseVelocity2d(
//                                new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), -gamepad1.right_stick_x / 3)
//                );
//            }



// Operator
//            if((gamepad2.right_stick_x>0 || gamepad2.right_stick_x <0) && isPickSample ){
//                SetWrist(90);
//            }
//
//            if((gamepad2.right_stick_y>0 || gamepad2.right_stick_y <0) && isPickSample){
//                SetWrist(0);
//            }
//
//            if((gamepad2.right_stick_x>0 && gamepad2.right_stick_y>0) && isPickSample){
//                SetWrist(45);
//            }
//
//            if((gamepad2.right_stick_x<0 && gamepad2.right_stick_y>0) && isPickSample){
//                SetWrist(135);
//            }


//            if(gamepad1.left_bumper && isBasket){ // DRIVER CONTROL
//                // HIGH BASKET
//                new SampleDropSeq(outtake);
//
//                isBasket = false;
//                isPickSample = false;
//            }
