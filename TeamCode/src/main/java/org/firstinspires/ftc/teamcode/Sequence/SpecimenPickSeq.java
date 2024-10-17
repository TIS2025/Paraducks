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

public class SpecimenPickSeq {
    public ArrayList<Action> ftc = new ArrayList<>();

    public SpecimenPickSeq(OuttakeSubsystem outtake, ElevatorSubsytem elevator) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN)),
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SPECIMEN_PICK)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SPECIMEN_PICK)),
                        new InstantAction(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.SPECIMEN_PICK)),
                        new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SPECIMEN_PICK)),
                        new InstantAction(()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.SPECIMEN_PICK))
                )
        );
    }
}
