package org.firstinspires.ftc.teamcode.InstantCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;

public class ElevatorCommand {
    public ElevatorCommand(ElevatorSubsytem elevator, ElevatorSubsytem.ElevateState state, int dropheight) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction( ()->elevator.updateState(state,dropheight))
                ));
    }
}
