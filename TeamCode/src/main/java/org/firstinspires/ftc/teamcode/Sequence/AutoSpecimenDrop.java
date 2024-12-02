package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackExtendCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.WristCommand;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class AutoSpecimenDrop {
    public AutoSpecimenDrop(OuttakeSubsystem outtake, ElevatorSubsytem elevator) {
        Actions.runBlocking(
                new SequentialAction(
//                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0)),
//                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
//                        new SleepAction(0.1),
                        new InstantAction(() -> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SPECIMEN_DROP)),
                        new InstantAction(() -> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SPECIMEN_DROP)),
                        new InstantAction(() -> new RackCommand(outtake, OuttakeSubsystem.RackState.SPECIMEN_DROP)),
                        new InstantAction(() -> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SPECIMEN_DROP)),
                        new SleepAction(0.2),
                        new InstantAction(() -> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.MAX_POS)),
                        new InstantAction(() -> new WristCommand(outtake, OuttakeSubsystem.WristState.SPECIMEN_DROP))
                )
        );
    }
}


