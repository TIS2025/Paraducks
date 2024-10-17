package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

import java.util.ArrayList;

public class HangSeq {
    public ArrayList<Action> ftc = new ArrayList<>();

    public HangSeq(OuttakeSubsystem outtake, ElevatorSubsytem elevator, ElevatorSubsytem.ElevateState state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_DROP)),
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP))
                )
        );
    }
}
