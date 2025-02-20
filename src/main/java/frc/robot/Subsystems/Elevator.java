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
    private final SparkMax LeftMotor = new SparkMax(ElevatorConstants.LMotorID, MotorType.kBrushless);
    private final SparkMax RightMotor = new SparkMax(ElevatorConstants.RMotorID, MotorType.kBrushless);

}
