package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.InstantCommand.ArmCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ElevatorCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.GripperCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.RackCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.ShoulderCommand;
import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsytem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class SamplePickSeq {
    public static Action SamplePickSeqAction(OuttakeSubsystem outtake, ElevatorSubsytem elevator) {
              return  new SequentialAction(
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE)),
                      new InstantAction(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK)),
                      new InstantAction(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HOME, 0)),
                        new InstantAction(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK)),
                        new InstantAction(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PICK)),
                        new InstantAction(() -> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
                );
    }

}



