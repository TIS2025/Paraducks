
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

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.AutoSpecimenDrop;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenPickSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

@Config
@Autonomous(name = "Auto_Red_Specimen_83", group = "Autonomous")
public class Auto_Red_Specimen_83 extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    private OuttakeSubsystem outtake;
    private ElevatorSubsytem elevator;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);

        Pose2d initialPose = new Pose2d(-21, 60, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, initialPose);


        //TODO ==================================================== Writing Trajectories ======================================================
        Action trajectoryAction0 = drive.actionBuilder(drive.pose)

                //TODO - Specimen Droping
                .stopAndAdd(()-> new AutoSpecimenDrop(outtake, elevator))
                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(-4, 36), Math.toRadians(-90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.4)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .stopAndAdd(()-> new InitSeq(outtake, elevator))


                .stopAndAdd(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK))
                .stopAndAdd(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK))
                .stopAndAdd(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.WRIST_RIGHT_DIAGONAL))
                .afterTime(0.8,()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.MAX_POS))
                .afterTime(0.6,()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))


                // First Sample Picking and Dropping
                .strafeToLinearHeading(new Vector2d(-44.5, 41), Math.toRadians(-120))
                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
                .waitSeconds(0.05)
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                .waitSeconds(0.15)
                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
                .strafeToLinearHeading(new Vector2d(-44.5, 48), Math.toRadians(120))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
                .waitSeconds(0.15)


                // Second Sample Picking and Dropping
                .strafeToLinearHeading(new Vector2d(-51, 41), Math.toRadians(-120))
                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
                .waitSeconds(0.15)
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                .waitSeconds(0.15)
                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
                .strafeToLinearHeading(new Vector2d(-53.5, 48), Math.toRadians(120))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
                .waitSeconds(0.15)


//                 Third Sample Picking and Dropping
                .strafeToLinearHeading(new Vector2d(-61, 40), Math.toRadians(-120))

                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
                .waitSeconds(0.05)
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                .waitSeconds(0.15)

                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))

                .strafeToLinearHeading(new Vector2d(-55, 48), Math.toRadians(120))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
                .waitSeconds(0.15)

                // Specimen PrePick
                .stopAndAdd(()-> new InitSeq(outtake, elevator))
                .waitSeconds(0.15)
                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
                .waitSeconds(0.15)
                .strafeToLinearHeading(new Vector2d(-38, 44), Math.toRadians(-90))
                // Specimen Pick
                .strafeToLinearHeading(new Vector2d(-38, 62), Math.toRadians(-90))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                .waitSeconds(0.1)
                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))

                // Specimen Hang
                .afterTime(0.6,()-> new AutoSpecimenDrop(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(-5, 34), Math.toRadians(-90))

                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.4)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .stopAndAdd(()-> new InitSeq(outtake, elevator))


                // Third Specimen Pick
                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(-38, 44), Math.toRadians(-90))

                // Specimen Pick
                .strafeToLinearHeading(new Vector2d(-38, 62), Math.toRadians(-90))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                .waitSeconds(0.1)
                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))
                .afterTime(0.6,()-> new AutoSpecimenDrop(outtake, elevator))


                // Specimen Hang
                .strafeToLinearHeading(new Vector2d(-7, 34), Math.toRadians(-90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.4)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .stopAndAdd(()-> new InitSeq(outtake, elevator))

                // Fourth Specimen Pick
                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(-40, 44), Math.toRadians(-90))

                // Specimen Pick
//                .strafeToLinearHeading(new Vector2d(38, -59), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-36.5, 62), Math.toRadians(-90))
                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                .waitSeconds(0.1)
                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))
                .afterTime(0.6,()-> new AutoSpecimenDrop(outtake, elevator))


                // Specimen Hang
                .strafeToLinearHeading(new Vector2d(-9, 34), Math.toRadians(-90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.4)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
                .stopAndAdd(()-> new InitSeq(outtake, elevator))



                .strafeToLinearHeading(new Vector2d(-420, 157), Math.toRadians(-90))


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

