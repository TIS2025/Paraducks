package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class SampleDropSeq{

    public static Action SampleDropSeqAction(OuttakeSubsystem outtake)
    {
               return  new SequentialAction(
                       new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                       new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_DROP)),
                       new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_DROP)),
                       new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SAMPLE_DROP)),
                       new SleepAction(0.5),
                       new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_DROP)),
                       new SleepAction(0.5),
                       new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SAMPLE_DROP))
               );


    }
}