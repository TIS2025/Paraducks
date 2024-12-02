package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.AutoSampleDrop;
import org.firstinspires.ftc.teamcode.Sequence.AutoSampleIntake;
import org.firstinspires.ftc.teamcode.Sequence.AutoSpecimenDrop;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

@Config
@Autonomous(name = "Auto_Red_Sample_37_Points", group = "Autonomous")
public class Auto_Red_Sample_37_Points extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    private OuttakeSubsystem outtake;
    private ElevatorSubsytem elevator;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);

        Pose2d initialPose = new Pose2d(21, 60, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, initialPose);


        //TODO ==================================================== Writing Trajectories ======================================================
        Action trajectoryAction0 = drive.actionBuilder(drive.pose)
                //TODO - Specimen Droping
//                .stopAndAdd(()-> new SpecimenDropSeq(outtake, elevator))
//                .strafeToLinearHeading(new Vector2d(9, 28), Math.toRadians(-90))
//                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))

                .stopAndAdd(()-> new AutoSpecimenDrop(outtake, elevator))
                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(9, 36), Math.toRadians(-90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.4)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .stopAndAdd(()-> new InitSeq(outtake, elevator))

                //TODO - Sample Picking
                .afterTime(0.5,()-> new AutoSampleIntake(outtake, elevator, 1))
                .strafeToLinearHeading(new Vector2d(55, 42.5), Math.toRadians(-90))
                .stopAndAdd(()-> new AutoSampleIntake(outtake))

                //TODO - Sample Outtake
                .strafeToLinearHeading(new Vector2d(58, 51), Math.toRadians(-135))
                .stopAndAdd(()-> new AutoSampleDrop(outtake, elevator))

                //TODO - Sample Picking
                .strafeToLinearHeading(new Vector2d(63, 42), Math.toRadians(-90))
                .stopAndAdd(()-> new AutoSampleIntake(outtake))


                //TODO - Sample Outtake
                .strafeToLinearHeading(new Vector2d(57, 52), Math.toRadians(-135))
                .stopAndAdd(()-> new AutoSampleDrop(outtake, elevator))
                .afterTime(0.4,()-> new AutoSampleIntake(outtake, elevator, 3))  // OverWriting
                .strafeToLinearHeading(new Vector2d(59, 40), Math.toRadians(-43))
                .stopAndAdd(()-> new AutoSampleIntake(outtake))

                .strafeToLinearHeading(new Vector2d(57, 52), Math.toRadians(-135))
                .stopAndAdd(()-> new AutoSampleDrop(outtake, elevator, 1))

                .afterTime(2,()-> new AutoSampleDrop(outtake, elevator, 2))
                .strafeToLinearHeading(new Vector2d(30, 5), Math.toRadians(-135))


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



        waitForStart();

        Actions.runBlocking(new SequentialAction(
                trajectoryAction0
        ));


    }
}

