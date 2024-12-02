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

public class AutoSampleIntake {
    public AutoSampleIntake(OuttakeSubsystem outtake, ElevatorSubsytem elevator, int i) {
        if(i==1){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK)),
                            new SleepAction(0.1),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_PICK)),
                            new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.MAX_POS)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))

                    ));
        }
        if(i==3){
            Actions.runBlocking(
                    new SequentialAction(
                            new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK)),
                            new SleepAction(0.1),
                            new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                            new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK)),
                            new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.WRIST_LEFT_DIAGONAL)),
                            new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.MAX_POS)),
                            new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))

                    ));
        }
    }

    public AutoSampleIntake(OuttakeSubsystem outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                        new SleepAction(0.1),
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK)),
                        new SleepAction(0.1),
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                        new SleepAction(0.2),
                        new InstantAction(()->new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.INIT))
                ));
    }



}



