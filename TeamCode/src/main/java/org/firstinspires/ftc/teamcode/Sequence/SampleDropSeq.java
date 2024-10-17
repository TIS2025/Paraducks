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

public class SampleDropSeq{

    public ArrayList<Action> ftc = new ArrayList<>();

    public SampleDropSeq(OuttakeSubsystem outtake,ElevatorSubsytem elevator, Boolean HighBasket)
    {

//        ftc.add(
        if(HighBasket){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_DROP)),
                            new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHBASKET, 0))
                    )
            );
        }
        else {
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_DROP)),
                            new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.LOWBASKET, 0))
                    )
            );
        }


    }


    public SampleDropSeq(OuttakeSubsystem outtake,ElevatorSubsytem elevator, ElevatorSubsytem.ElevateState state)
    {
//        ftc.add(
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                        new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_DROP)),
                        new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.LOWBASKET, 0))

                )
        );


    }
}