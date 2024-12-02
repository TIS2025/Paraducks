package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.Sequence.LimeSamplePickSequence;
import org.firstinspires.ftc.teamcode.Sequence.SampleDropSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Extension test")
@Config
public class ExtTest extends LinearOpMode {

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


    // TODO ==================================================== Drive variables ==============================================================
    public ElapsedTime esp;
    public double strafe = 1, speed = 1, turn = 1;



    // TODO =================================================== Into the deep Variables ===========================================
    Limelight3A limelight;

    public List<Sample>
            redSamples = new ArrayList<>();
    public List<Sample> blueSamples = new ArrayList<>();
    public List<Sample> yellowSamples = new ArrayList<>();
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
    KinematicSolver solver = new KinematicSolver(2.5,0.5,0);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        OuttakeSubsystem outtake = new OuttakeSubsystem(robot);
        ElevatorSubsytem elevator = new ElevatorSubsytem(robot);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        telemetry.setMsTransmissionInterval(100);
        limelight.pipelineSwitch(2);

        Gamepad C =new Gamepad();
        Gamepad P = new Gamepad();

        if(opModeInInit())
        {
            new InitSeq(outtake, elevator);
        }


        waitForStart();
        limelight.start();
        while (opModeIsActive()){

            ftc = updateAction();
            botHeading = drive.pose.heading.toDouble();
            P.copy(C);
            C.copy(gamepad1);
            redSamples.clear();
            blueSamples.clear();
            yellowSamples.clear();
            LLResult result = limelight.getLatestResult();
            sample_filter(result);


            if(gamepad1.y){ // TODO YELLOW PICK SEQUNCE

                ftc.add(
                        new SequentialAction(
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
            if (gamepad1.a) { // Dropped
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

            if(gamepad1.x){ // TODO BLUE PICK SEQUNCE

                ftc.add(
                        new SequentialAction(
                                new InstantAction(()->{
                                    if(!blueSamples.isEmpty()){
                                        pickup_pos = solver.getExt(blueSamples.get(0).field_pos);
                                        obj_orient_blue = blueSamples.get(0).orientation;
                                        pos = obj_orient_blue?0.4339:0.715;
                                    }

                                }),
                                new InstantAction(()-> outtake.setRackPosInches(pickup_pos)),
                                new InstantAction(()-> outtake.WristServo(pos)),

                                LimeSamplePickSequence.LimeSamplePickAction(outtake, elevator),
//                                new SleepAction(0.5),

                                new InstantAction(()-> robot.Arm.setPosition(Globals.armSamplePick)),
//                                new SleepAction(0.2),
                                new InstantAction(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePrePick)),
                                new SleepAction(0.2),
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
                                new SleepAction(0.3),
                                new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                                new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                                new InstantAction(() -> SamplePicked = true),
                                new InstantAction(() -> isGripSample = false),
                                new InstantAction(() -> speed = 1),
                                new InstantAction(() -> strafe = 1),
                                new InstantAction(() -> turn = 1),
                                new InstantAction(()-> robot.Arm.setPosition(0.2)),
                                new InstantAction(() -> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                                new InstantAction(() -> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.INIT)),
                                SampleDropSeq.SampleDropSeqAction(outtake)

                        )
                );

            }

            if(gamepad1.b){ // TODO RED PICK SEQUNCE

                ftc.add(
                        new SequentialAction(
                                new InstantAction(()->{
                                    if(!redSamples.isEmpty()){
                                        pickup_pos = solver.getExt(redSamples.get(0).field_pos);
                                        obj_orient_red = redSamples.get(0).orientation;
                                        pos = obj_orient_red?0.4339:0.715;
                                    }

                                }),
                                new InstantAction(()-> outtake.setRackPosInches(pickup_pos)),
                                new InstantAction(()-> outtake.WristServo(pos)),

                                LimeSamplePickSequence.LimeSamplePickAction(outtake, elevator),
//                                new SleepAction(0.5),

                                new InstantAction(()-> robot.Arm.setPosition(Globals.armSamplePick)),
                                new InstantAction(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePrePick)),
                                new SleepAction(0.2),
                                new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose)),
                                new SleepAction(0.3),
                                new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                                new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                                new InstantAction(() -> SamplePicked = true),
                                new InstantAction(() -> isGripSample = false),
                                new InstantAction(() -> speed = 1),
                                new InstantAction(() -> strafe = 1),
                                new InstantAction(() -> turn = 1),
                                new InstantAction(()-> robot.Arm.setPosition(0.2)),
                                new InstantAction(() -> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                                new InstantAction(() -> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.INIT)),
                                SampleDropSeq.SampleDropSeqAction(outtake)

                        )
                );

            }

//            if(gamepad1.left_bumper){
//                ftc.add(
//                        new SequentialAction(
//                                SampleDropSeq.SampleDropSeqAction(outtake)
//                        )
//                );
//            }
//
//            if(gamepad1.right_bumper){
//                ftc.add(
//                        new SequentialAction(
//                                new InstantAction(() -> robot.Gripper.setPosition(Globals.gripperOpen)),
//                                new SleepAction(0.6),
//                                InitSeq.InitSeqAction(outtake, elevator)
//                        )
//                );
//            }


            drive.setDrivePowers(
            new PoseVelocity2d(
            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x / 2)
                );

            if (gamepad1.left_trigger>0.3){
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2), -gamepad1.right_stick_x / 3)
                );
            }


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

}
