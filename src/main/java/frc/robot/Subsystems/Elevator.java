// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
                                        // initialize in the constructor
    private final SparkMax LeftMotor = new SparkMax(ElevatorConstants.LMotorID, MotorType.kBrushless);
    private final SparkMax RightMotor = new SparkMax(ElevatorConstants.RMotorID, MotorType.kBrushless);
    private final SparkClosedLoopController leftPID = LeftMotor.getClosedLoopController();
    private final SparkClosedLoopController rightPID = RightMotor.getClosedLoopController();
    // make a profiled PID controller and add a gravity FF
    //private final ProfiledPIDController pid;
    //private final ElevatorFeedforward ff;
    public void SetLevel(Integer Level){
        if (Level == 1) goToHeight(ElevatorConstants.Level1);
        if (Level == 2) goToHeight(ElevatorConstants.Level2);
        if (Level == 3) goToHeight(ElevatorConstants.Level3);
        if (Level == 4) goToHeight(ElevatorConstants.Level4);
    }
    // use SparkBaseConfig here
public Elevator () {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();


    leftConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .inverted(true)
    .encoder.positionConversionFactor(1/ElevatorConstants.gearing)
        .velocityConversionFactor(1/ElevatorConstants.gearing/60);
    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .maxMotion.maxVelocity(ElevatorConstants.maxVelocity)
            .maxAcceleration(ElevatorConstants.maxAcceleration);

    rightConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .inverted(false)
    .encoder.positionConversionFactor(1/ElevatorConstants.gearing)
        .velocityConversionFactor(1/ElevatorConstants.gearing/60);
    rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .maxMotion.maxVelocity(ElevatorConstants.maxVelocity)
            .maxAcceleration(ElevatorConstants.maxAcceleration);

    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

} //do setup here, use the gear ratios/

public void goToHeight (double height) {
leftPID.setReference(height,ControlType.kPosition,
    ClosedLoopSlot.kSlot0, ElevatorConstants.gravityCompensator, ArbFFUnits.kVoltage);
rightPID.setReference(height,ControlType.kPosition,
    ClosedLoopSlot.kSlot0, ElevatorConstants.gravityCompensator, ArbFFUnits.kVoltage);
}
}

