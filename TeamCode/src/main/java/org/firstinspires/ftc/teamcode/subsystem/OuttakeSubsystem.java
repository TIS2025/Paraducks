package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class OuttakeSubsystem {
    private RobotHardware robot;
    ShoulderState shoulderStatee=ShoulderState.INIT;
    ////////////
    public GripperState gripperState=GripperState.GRIP_CLOSE;
    public RackState rackState=RackState.INIT;

    public  ArmState armState = ArmState.INIT;
    public WristState wristState = WristState.INIT;
    public RackExtendState rackExtendState = RackExtendState.INIT;

    // profiling
    public double totalAngle = 0;
    public double prevAngle = 0;

    public OuttakeSubsystem(RobotHardware robot) {

        this.robot = robot;
    }

    public enum ShoulderState{
        INIT,
        SAMPLE_PICK,
        SAMPLE_DROP,
        SAMPLE_PRE_PICK,
        AUTO_SAMPLE_PRE_PICK,
        SPECIMEN_PICK,
        SPECIMEN_DROP,
        AUTO_SPECIMEN_DROP,
        LIME_SAMPLE_PICK
    }

    public enum GripperState{
        GRIP_OPEN,
        GRIP_CLOSE,
        GRIP_SAFE,
        GRIP_LIME_OPEN
    }

    public enum RackState{
        INIT,
        SAMPLE_PICK,
        SAMPLE_DROP,
        AUTO_SAMPLE_DROP,

        SPECIMEN_PICK,
        SPECIMEN_DROP,

    }

    public enum RackExtendState{
        INIT,
        SAMPLE_PICK,
        SAMPLE_DROP,
        SPECIMEN_PICK,
        SPECIMEN_DROP,
        MID_POS,
        MAX_POS
    }

    public enum ArmState{
        INIT,
        SAMPLE_PICK,
        SAMPLE_DROP,

        LIME_SAMPLE_PICK,
        SPECIMEN_PICK,
        SPECIMEN_DROP,
        AUTO_SPECIMEN_DROP,

    }


    public enum WristState{
        INIT,
        SAMPLE_PICK,
        SAMPLE_DROP,

        WRIST_LEFT_DIAGONAL,
        WRIST_RIGHT_DIAGONAL,

        SPECIMEN_PICK,
        SPECIMEN_DROP,
    }


    //ShoulderState
    public void updateState(ShoulderState state){
        this.shoulderStatee=state;
        switch (state){
            case INIT:
                setServoShoulder(Globals.shoulderInit);
                break;
            case SAMPLE_PICK:
                setServoShoulder(Globals.shoulderSamplePick);
                break;
            case LIME_SAMPLE_PICK:
                setServoShoulder(Globals.shoulderLimeSamplePick);
                break;
            case SAMPLE_DROP:
                setServoShoulder(Globals.shoulderSampleDrop);
                break;
            case SPECIMEN_PICK:
                setServoShoulder(Globals.shoulderSpecimenPick);
                break;
            case SAMPLE_PRE_PICK:
                setServoShoulder(Globals.shoulderSamplePrePick);
                break;
            case AUTO_SAMPLE_PRE_PICK:
                setServoShoulder(Globals.AutoshoulderSamplePick);
                break;
            case SPECIMEN_DROP:
                setServoShoulder(Globals.shoulderSpecimenDrop);
                break;
            case AUTO_SPECIMEN_DROP:
                setServoShoulder(Globals.AutoshoulderSpecimenDrop);
                break;

        }
    }

    public void updateState(ArmState state){
        this.armState=state;
        switch (state){
            case INIT:
                ArmServo(Globals.armInit);
                break;
            case SAMPLE_PICK:
                ArmServo(Globals.armSamplePick);
                break;
            case SAMPLE_DROP:
                ArmServo(Globals.armSampleDrop);
                break;
            case LIME_SAMPLE_PICK:
                ArmServo(Globals.limeSamplePick);
                break;
            case SPECIMEN_PICK:
                ArmServo(Globals.armSpecimenPick);
                break;
            case SPECIMEN_DROP:
                ArmServo(Globals.armSpecimenDrop);
                break;
            case AUTO_SPECIMEN_DROP:
                ArmServo(Globals.AutoarmSpecimenDrop);
                break;
        }
    }

    public void updateState(WristState state){
        this.wristState=state;
        switch (state){
            case INIT:
                WristServo(Globals.wristInit);
                break;
            case SAMPLE_PICK:
                WristServo(Globals.wristSamplePick);
                break;
            case SAMPLE_DROP:
                WristServo(Globals.wristSampleDrop);
                break;
            case SPECIMEN_PICK:
                WristServo(Globals.wristSpecimenPick);
                break;
            case SPECIMEN_DROP:
                WristServo(Globals.wristSpecimenDrop);
                break;
            case WRIST_LEFT_DIAGONAL:
                WristServo(Globals.WristLeftDiagonal);
                break;
            case WRIST_RIGHT_DIAGONAL:
                WristServo(Globals.WristRightDiagonal);
                break;

        }
    }



    //Gripper
    public void updateState(GripperState state){
        this.gripperState=state;
        switch (state){
            case GRIP_OPEN:                           //BOTH
                robot.Gripper.setPosition(Globals.gripperOpen);
                break;
            case GRIP_CLOSE:
                robot.Gripper.setPosition(Globals.gripperClose);
                break;
            case GRIP_SAFE:
                robot.Gripper.setPosition(Globals.gripperSafe);
                break;
            case GRIP_LIME_OPEN:
                robot.Gripper.setPosition(Globals.gripperLimeOpen);
                break;
        }
    }

    //RACK STATE
    public void updateState(RackState state){
        this.rackState=state;
        switch (state){
            case INIT:
                robot.RackExtend.setPosition(Globals.rackInit);
                break;
            case SAMPLE_PICK:
                robot.RackExtend.setPosition(Globals.rackSamplePick);
                break;
            case SAMPLE_DROP:
                robot.RackExtend.setPosition(Globals.rackSampleDrop);
                break;
            case AUTO_SAMPLE_DROP:
                robot.RackExtend.setPosition(Globals.autoRackSampleDrop);
                break;
            case SPECIMEN_PICK:
                robot.RackExtend.setPosition(Globals.rackSpecimenPick);
                break;
            case SPECIMEN_DROP:
                robot.RackExtend.setPosition(Globals.rackSpecimenDrop);
                break;
        }
    }

    public void updateState(RackExtendState state){
        this.rackExtendState=state;
        switch (state){
            case INIT:
                setRackPos(Globals.rackExtensionRightInit);
                break;
            case SAMPLE_PICK:
                setRackPos(Globals.rackExtensionRightSamplePIck);
                break;
            case SAMPLE_DROP:
                setRackPos(Globals.rackExtensionRightSampleDrop);
                break;
            case SPECIMEN_PICK:
                setRackPos(Globals.rackExtensionRightSpecimenPIck);
                break;
            case SPECIMEN_DROP:
                setRackPos(Globals.rackExtensionRightSpecimenDrop);
                break;
            case MID_POS:
                setRackPos(Globals.rackExtensionMidPos);
                break;
            case MAX_POS:
                setRackPos(Globals.rackExtensionMaxPos);
                break;

        }
    }

    public void ArmServo(double pos){
        robot.Arm.setPosition(pos);
    }

    public void WristServo(double pos){
        robot.Wrist.setPosition(pos);
    }

    public void setServoShoulder(double pos){

        robot.Shoulder.setPosition(pos);
    }
    public void setServoShoulder(double pos, double speed){
        totalAngle = pos*speed + prevAngle*(1-speed);
        robot.Shoulder.setPosition(totalAngle);
    }

    public void updateShoulderServo(){
        prevAngle = totalAngle;
    }
    public void setRackPos(double pos){
        robot.RackExtensionRight.setPosition(pos);
        robot.RackExtensionLeft.setPosition(1 - (pos-0.03));

    }

    public void setRackPosInches(double inch){
        inch = Math.max(Math.min(inch,10),1.5);
        double pos = 0.00373447*inch*inch -0.09780091*inch + 1.00738517;
        robot.RackExtensionRight.setPosition(pos);
        robot.RackExtensionLeft.setPosition(1 - (pos-0.03));
    }




//    public void RunCServo(double time){
//        if(checkRack){
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new InstantAction(()-> robot.RackExtend.setPower(1)),
//                            new SleepAction(time),
//                            new InstantAction(()-> robot.RackExtend.setPower(0))
//                    )
//            );
//        }
//    }
//
//
//    public void RunCServoReverse(double time){
//
//        if(!checkRack){
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new InstantAction(()-> robot.RackExtend.setPower(-1)),
//                            new SleepAction(time),
//                            new InstantAction(()-> robot.RackExtend.setPower(0))
//                    )
//            );
//        }
//    }



}
