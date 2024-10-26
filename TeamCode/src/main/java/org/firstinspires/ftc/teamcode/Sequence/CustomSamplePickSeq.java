package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.CustomRackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

import java.util.ArrayList;

public class CustomSamplePickSeq {

    public ArrayList<Action> ftc = new ArrayList<>();

    public CustomSamplePickSeq(OuttakeSubsystem outtake, ElevatorSubsytem elevator,double pos) {
//        ftc.clear();
//            ftc.add(
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK)),
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                        new InstantAction(() -> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK)),
                        new InstantAction(() -> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_PICK)),
                        new InstantAction(() -> new CustomRackExtendCommand(outtake, pos)),
                        new InstantAction(() -> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PICK)),
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
                )
        );
    }
}



