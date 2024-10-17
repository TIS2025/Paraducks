package org.firstinspires.ftc.teamcode.InstantCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class WristCommand {

    public WristCommand(OuttakeSubsystem Outake, OuttakeSubsystem.WristState state) {
        Actions.runBlocking(new SequentialAction(
                new InstantAction( ()->Outake.updateState(state))
        ));
    }
}
