package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.KinematicSolver;
import org.firstinspires.ftc.teamcode.utils.PerspectiveSolver;
import org.firstinspires.ftc.teamcode.utils.Sample;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Auto_Lime_Light", group = "Autonomous")
public class AutoLimeLight extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    private OuttakeSubsystem outtake;
    private ElevatorSubsytem elevator;


    // Todo =========================== Lime Light =====================================
    public List<Sample>
            redSamples = new ArrayList<>();
    public List<Sample> blueSamples = new ArrayList<>();
    public List<Sample> yellowSamples = new ArrayList<>();
    //    Sample x = new Sample();
    int CAMERA_HEIGHT = 480;
    int CAMERA_WIDTH = 640;
    public static Action trajectoryAction1 = null;
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
    Point field_pos = null;

    double y_Offset_val = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);

//        Pose2d initialPose = new Pose2d(-9, -60, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);

        telemetry.setMsTransmissionInterval(100);
        robot.limelight.pipelineSwitch(2);
        yellowSamples.clear();



        //TODO ==================================================== Writing Trajectories ======================================================
        Action trajectoryAction0 = drive.actionBuilder(drive.pose)
//                //TODO - Specimen Droping
//
//                .stopAndAdd(()-> new AutoSpecimenDrop(outtake, elevator))
//                .waitSeconds(0.4)
//                .strafeToLinearHeading(new Vector2d(9, 36), Math.toRadians(-90))
//                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
//                .waitSeconds(0.4)
//                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                .stopAndAdd(()-> new InitSeq(outtake, elevator))
//
//                //TODO - Sample Picking
//                .afterTime(0.5,()-> new AutoSampleIntake(outtake, elevator, 1))
//                .strafeToLinearHeading(new Vector2d(55, 42.5), Math.toRadians(-90))
//                .stopAndAdd(()-> new AutoSampleIntake(outtake))
//
//                //TODO - Sample Outtake
//                .strafeToLinearHeading(new Vector2d(58, 51), Math.toRadians(-135))
//                .stopAndAdd(()-> new AutoSampleDrop(outtake, elevator))
//
//                //TODO - Sample Picking
//                .strafeToLinearHeading(new Vector2d(63, 42), Math.toRadians(-90))
//                .stopAndAdd(()-> new AutoSampleIntake(outtake))
//
//
//                //TODO - Sample Outtake
//                .strafeToLinearHeading(new Vector2d(57, 52), Math.toRadians(-135))
//                .stopAndAdd(()-> new AutoSampleDrop(outtake, elevator))
//                .afterTime(0.4,()-> new AutoSampleIntake(outtake, elevator, 3))  // OverWriting
//                .strafeToLinearHeading(new Vector2d(59, 40), Math.toRadians(-43))
//                .stopAndAdd(()-> new AutoSampleIntake(outtake))
//
//                .strafeToLinearHeading(new Vector2d(57, 52), Math.toRadians(-135))
//                .stopAndAdd(()-> new AutoSampleDrop(outtake, elevator, 1))
//
//                .afterTime(2,()-> new AutoSampleDrop(outtake, elevator, 2))
//                .strafeToLinearHeading(new Vector2d(30, 5), Math.toRadians(-135))


                // TODO =========================== Going For Lime Light =========================

                .stopAndAdd(()->{
                    for (int i=0;i<5;i++){
                        LLResult result = robot.limelight.getLatestResult();
                        sample_filter(result);
                        telemetry.update();
                    }
                })

                .waitSeconds(0.5)

                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .stopAndAdd(()-> {
                    telemetry.addData("val",y_Offset_val);
                    telemetry.addData("PosX",drive.pose.position.x);
                    telemetry.addData("PosY",drive.pose.position.y);
                    telemetry.update();
                    drive.updatePoseEstimate();
                })
                .waitSeconds(2)
                .build();

        telemetry.addData("y-offset-val",y_Offset_val);

//        Action trajectoryAction1 = drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(new Vector2d(0, 0.1+y_Offset_val), Math.toRadians(0))
//                .build();

        update();

        Action trajectoryAction2 = drive.actionBuilder(drive.pose)
                .stopAndAdd(()->{
                    if(!yellowSamples.isEmpty()){
                        pickup_pos = solver.getExt(yellowSamples.get(0).field_pos);
                        obj_orient_yellow = yellowSamples.get(0).orientation;
                        pos = obj_orient_yellow?0.4339:0.715;
                    }
                })

                .waitSeconds(1)
                .stopAndAdd(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK))
                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0))
                .stopAndAdd(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK))
                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PICK))
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .waitSeconds(0.5)

                .stopAndAdd(()-> outtake.setRackPosInches(pickup_pos))
                .stopAndAdd(()-> outtake.WristServo(pos))
                .waitSeconds(1)

                .stopAndAdd(()-> robot.Shoulder.setPosition(Globals.shoulderSamplePrePick))
                .waitSeconds(0.1)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperClose))
                .waitSeconds(0.2)

                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP))
                .stopAndAdd(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_DROP))
                .stopAndAdd(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP))
                .waitSeconds(0.5)
                .stopAndAdd(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP))
                .waitSeconds(0.5)
                .stopAndAdd(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP))
                .waitSeconds(20)
                .build();



        if (opModeInInit())
        {
            telemetry.addLine("Init");
            Actions.runBlocking(new SequentialAction(
                    new InstantAction(()-> new InitSeq(outtake, elevator)),
                    new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperOpen)),
                    new SleepAction(3),
                    new InstantAction(()-> robot.Gripper.setPosition(Globals.gripperClose))
            ));
        }
        drive.updatePoseEstimate();
        waitForStart();
        robot.limelight.start();
        update();

        Actions.runBlocking(new SequentialAction(
                trajectoryAction0,
//                new InstantAction(()-> update()),
//                trajectoryAction1,
                trajectoryAction2
        ));


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

                field_pos = Psolver.getX2Y2(new Point(cx, cy));
                boolean orientation = width/height>1.3    ;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();
                double offset = orientation?0.5:1.5;

//                if (Math.abs(field_pos.y) <1.5 && field_pos.x < 12 && class_id == 0) {
//                    field_pos.x+=offset;
//                    y_Offset_val = field_pos.y;
//                    blueSamples.add(new Sample(field_pos,class_id,confidence,orientation));
//                }
//                if (Math.abs(field_pos.y) <1.5 && field_pos.x < 12 && class_id == 1) {
//                    field_pos.x+=offset;
//                    y_Offset_val = field_pos.y;
//                    redSamples.add(new Sample(field_pos,class_id,confidence,orientation));
//                }
                if (Math.abs(field_pos.y) < 10 && field_pos.x < 12 && class_id == 2) {
                    field_pos.x+=offset;
                    y_Offset_val = field_pos.y;
                    yellowSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                telemetry.addData("Field_Pos",field_pos);
            }
        }
        catch (Exception e){
            telemetry.addLine("Failed to input samples");
            telemetry.addData("e ",e);
        }
    }



    public void update(){
         trajectoryAction1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(0, 0.1+ y_Offset_val), Math.toRadians(0))
                 .stopAndAdd(()-> {
                     drive.updatePoseEstimate();
                     telemetry.addData("val update",y_Offset_val);
                     telemetry.addData("PosX",drive.pose.position.x);
                     telemetry.addData("PosY",drive.pose.position.y);
                     telemetry.update();
                 })
                .build();

    }
}

