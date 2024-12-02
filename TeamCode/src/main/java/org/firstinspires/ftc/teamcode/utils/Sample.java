package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.Point;

public class Sample {
    public double confidence;
    public int class_id;
    public Point field_pos;
    public boolean orientation;

    public Sample(Point field_pos,int class_id,double confidence,boolean orientation){
        this.field_pos = field_pos;
        this.class_id = class_id;
        this.confidence = confidence;
        this.orientation = orientation;
    }
}