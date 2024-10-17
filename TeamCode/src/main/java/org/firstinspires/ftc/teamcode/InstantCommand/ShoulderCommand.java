package org.firstinspires.ftc.teamcode.InstantCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class ShoulderCommand {
    public ShoulderCommand(OuttakeSubsystem outake, OuttakeSubsystem.ShoulderState state) {
        Actions.runBlocking(new SequentialAction(
                new InstantAction( ()->outake.updateState(state))
        ));
    }
}
