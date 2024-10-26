package org.firstinspires.ftc.teamcode.InstantCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class CustomRackExtendCommand {
    public CustomRackExtendCommand(OuttakeSubsystem outtake, double pos) {
        Actions.runBlocking(new SequentialAction(
                new InstantAction( ()->outtake.CustomRackState(pos))
        ));
    }
}
