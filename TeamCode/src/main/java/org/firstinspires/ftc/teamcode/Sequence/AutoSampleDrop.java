package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

public class AutoSampleDrop{
    public AutoSampleDrop(OuttakeSubsystem outtake, ElevatorSubsytem elevator)
    {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHBASKET, 0)),
                        new InstantAction(()->new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.AUTO_SAMPLE_DROP)),
                        new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                        new SleepAction(2.7), // 2.8
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                        new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                        new SleepAction(0.4), // 0.5
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                        new SleepAction(0.1),
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK)),
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK)),
                        new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_PICK)),
                        new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.MAX_POS)),
                        new SleepAction(2.7), //2.8
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))

                )


        );

    }


    public AutoSampleDrop(OuttakeSubsystem outtake, ElevatorSubsytem elevator, int i)
    {
        if(i==1){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHBASKET, 0)),
                            new InstantAction(()->new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.AUTO_SAMPLE_DROP)),
                            new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                            new SleepAction(2.7),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                            new SleepAction(0.4),
                            new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                            new SleepAction(0.1),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0))
                    )


            );
        }
        if(i==2){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.INIT)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.INIT)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PICK)),
                            new InstantAction(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
                    )
            );
        }
    }



    public AutoSampleDrop(OuttakeSubsystem outtake, ElevatorSubsytem elevator, int i, char c)
    {
        if(i==1){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                            new InstantAction(()->new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.AUTO_SAMPLE_DROP)),
                            new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP)),
                            new SleepAction(0.4),
                            new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                            new SleepAction(0.1),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT)),
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.INIT)),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0))
                    )


            );
        }
        if(i==2){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.INIT)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.INIT)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PICK))
                    )
            );
        }
    }
}