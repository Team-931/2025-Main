// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
                                        // initialize in the constructor
    private final SparkMax LeftMotor = new SparkMax(ElevatorConstants.LMotorID, MotorType.kBrushless);
    private final SparkMax RightMotor = new SparkMax(ElevatorConstants.RMotorID, MotorType.kBrushless);

    // make a profiled PID controller and add a gravity FF
    public void SetLevel(Integer Level){
        if (Level == 1) goToHeight(ElevatorConstants.Level1);
        if (Level == 2) goToHeight(ElevatorConstants.Level2);
        if (Level == 3) goToHeight(ElevatorConstants.Level3);
        if (Level == 4) goToHeight(ElevatorConstants.Level4);
    }
    // use SparkBaseConfig here
public Elevator () {} //do setup here, use the gear ratios/

public void goToHeight (double height) {

}
}

