package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

import java.util.ArrayList;

public class InitSeq {
    public ArrayList<Action> runningActions = new ArrayList<>();

    public InitSeq(OuttakeSubsystem outtake, ElevatorSubsytem elevator) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                        new InstantAction(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.INIT)),
                        new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.INIT)),
                        new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.INIT)),
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0))
                )
        );
    }
    public static Action InitSeqAction(OuttakeSubsystem outtake, ElevatorSubsytem elevator) {
               return new SequentialAction(
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                        new InstantAction(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.INIT)),
                        new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.INIT)),
                        new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.INIT)),
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0))
                );
    }




}
