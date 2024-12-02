package org.firstinspires.ftc.teamcode.subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ElevatorSubsytem {

    private RobotHardware robot;
    public ElevateState elevateState=ElevateState.HOME;

    public enum ElevateState{
        HOME,
        HIGHBASKET,
        LOWBASKET,
        HIGHRUNG,
        LOWRUNG,
        HANGERPOS,
        HANG,
        SPECIMENHANGED,
        SPECIMENPICKED

    }



    public ElevatorSubsytem(RobotHardware robot) {
        this.robot = robot;
    }


    public void updateState(ElevateState state,int DropConstant){
        this.elevateState=state;
        switch (state){
            case HOME:
                extendTo(Globals.home,1);
                break;
            case HIGHBASKET:
                extendTo(Globals.highBasket+DropConstant,1);
                break;
            case LOWBASKET:
                extendTo(Globals.lowBasket+DropConstant,1);
                break;
            case HIGHRUNG:
                extendTo(Globals.highRung+DropConstant,1);
                break;
            case LOWRUNG:
                extendTo(Globals.lowrung+DropConstant,1);
                break;
            case SPECIMENHANGED:
                extendTo(Globals.specimenHanged+DropConstant, 1);
                break;
            case SPECIMENPICKED:
                extendTo(Globals.specimenPicked+DropConstant, 1);
                break;
            case HANGERPOS:
                extendTo(Globals.hangpos, 1);
                break;
            case HANG:
                extendTo(Globals.hang, 1);
                break;
        }
    }

    public void extendTo(int targetPosition, double power){
        robot.RightElevator.setTargetPosition(-targetPosition);
        robot.RightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RightElevator.setPower(power);

        robot.LeftElevator.setTargetPosition(targetPosition);
        robot.LeftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LeftElevator.setPower(power);
    }
}