package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.Point;

public class KinematicSolver {
    double arm_l1;
    double x_offset;
    double y_offset;

    public KinematicSolver(double arm_l1, double x_offset, double y_offset){
        this.arm_l1 = arm_l1;
        this.x_offset = x_offset;
        this.y_offset = y_offset;
    }

    //takes target point as input and gives extension in inches and yaw in degrees as element 0 and 1 of the array
    public double getExt(Point target){
        double ext = 0;
            ext = target.x - x_offset - arm_l1;
        return ext;
    }
}
