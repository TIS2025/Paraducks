
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.AutoSampleSpecimenDrop;
import org.firstinspires.ftc.teamcode.Sequence.AutoSampleSpecimenIntake;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpecimenPickSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

@Config
@Disabled
@Deprecated
@Autonomous(name = "Auto_Blue_Specimen_33_points", group = "Autonomous")
public class Auto_Blue_Specimen_33_points extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    private OuttakeSubsystem outtake;
    private ElevatorSubsytem elevator;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(robot);
        elevator = new ElevatorSubsytem(robot);

        Pose2d initialPose = new Pose2d(21, -60, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);


        //TODO ==================================================== Writing Trajectories ======================================================
        Action trajectoryAction0 = drive.actionBuilder(drive.pose)

                //TODO - Specimen Droping
                .stopAndAdd(()-> new SpecimenDropSeq(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(5, -25), Math.toRadians(90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.5)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))

                .strafeToLinearHeading(new Vector2d(5, -35), Math.toRadians(90))

                //TODO - Sample Picking
                .afterTime(0.5,()-> new AutoSampleSpecimenIntake(outtake, elevator, 1))
//                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(52, -43.5), Math.toRadians(90))
                .stopAndAdd(()-> new AutoSampleSpecimenIntake(outtake))
                .afterTime(0.7, ()-> new AutoSampleSpecimenDrop(outtake, elevator, 1, 's'))



                .strafeToLinearHeading(new Vector2d(63, -42), Math.toRadians(90))
                .stopAndAdd(()-> new AutoSampleSpecimenIntake(outtake, elevator, 1))
                .afterTime(0.6,()-> new AutoSampleSpecimenIntake(outtake))
                .strafeToLinearHeading(new Vector2d(60, -44), Math.toRadians(90))
                .waitSeconds(1)
                .stopAndAdd(()-> new AutoSampleSpecimenDrop(outtake, elevator, 1, 's'))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(36, -44), Math.toRadians(90))
                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90))
                .waitSeconds(0.01)
                .stopAndAdd(()-> new SpecimenDropSeq(outtake, elevator))

                .strafeToLinearHeading(new Vector2d(7, -26), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(5, -26), Math.toRadians(90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.5)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))

                .strafeToLinearHeading(new Vector2d(5, -31), Math.toRadians(90))
                // Todo Second pick
                .splineToConstantHeading(new Vector2d(38, -39), Math.toRadians(90))

                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(38, -60), Math.toRadians(90))
                .waitSeconds(0.01)
                .stopAndAdd(()-> new SpecimenDropSeq(outtake, elevator))

                .strafeToLinearHeading(new Vector2d(0, -26), Math.toRadians(90))
                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
                .waitSeconds(0.5)
                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))

                .afterTime(1,()-> new InitSeq(outtake, elevator))
                .strafeToLinearHeading(new Vector2d(42, -57), Math.toRadians(90))
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

