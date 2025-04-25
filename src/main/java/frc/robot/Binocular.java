// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import frc.robot.Constants.BinocularConstants;
import frc.robot.LimelightHelpers.RawFiducial;
/** Add your docs here. */
public class Binocular {
    private LimelightHelpers.RawFiducial[] leftResults = new RawFiducial[BinocularConstants.targetCount], 
    rightResults = new RawFiducial[BinocularConstants.targetCount];
    public void readTargets() {
        Arrays.fill(leftResults, null);
        Arrays.fill(rightResults, null);
        for (var r : LimelightHelpers.getRawFiducials(BinocularConstants.lcam)) leftResults [r.id - 1] = r;
        for (var r : LimelightHelpers.getRawFiducials(BinocularConstants.rcam)) rightResults [r.id - 1] = r;
    }

    double getRightDist(int id) {
        var r =  rightResults [id-1];
        if (null != r) 
            return r.distToCamera;
        else return -1;
        
    }

    double forwardDist(int id) {
        var r = rightResults [id-1];
        var l = leftResults [id-1];
        if (null != r && null != l)
        return BinocularConstants.width/(tan(l.txnc) - tan(r.txnc));
        return -1;
    }

    double tan(double x) {
        return Math.tan(Math.toRadians(x));
    }
}
