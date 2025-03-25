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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
                                        // initialize in the constructor
    private final SparkMax LeftMotor = new SparkMax(ElevatorConstants.LMotorID, MotorType.kBrushless);
    private final SparkMax RightMotor = new SparkMax(ElevatorConstants.RMotorID, MotorType.kBrushless);
    private final SparkClosedLoopController leftPID = LeftMotor.getClosedLoopController();
    //private final SparkClosedLoopController rightPID = RightMotor.getClosedLoopController();

    // reads false when hit
    StatusSignal<ForwardLimitValue> upperLimit; // must be refreshed before reading
    
    DutyCycleEncoder absEncoder = new DutyCycleEncoder(9);
    
    DigitalInput lowerLimit = new DigitalInput(0); //reads false when hit

// make a profiled PID controller and add a gravity FF
    //private final ProfiledPIDController pid;
    //private final ElevatorFeedforward ff;
    public void SetLevel(Integer Level){
        if (Level == 1) {
            goToHeight(ElevatorConstants.Level1);
        }
        if (Level == 2) {
            goToHeight(ElevatorConstants.Level2);
        }
        if (Level == 3) {
            goToHeight(ElevatorConstants.Level3);
        }
        if (Level == 4) {
            goToHeight(ElevatorConstants.Level4);
        }
        if (Level == 5) {
            goToHeight(ElevatorConstants.BargeLevel);
        }
    }
    // use SparkBaseConfig here
public Elevator (StatusSignal<ForwardLimitValue> statusSignal) {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    upperLimit = statusSignal;
    
    leftConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .inverted(true)
    .encoder.positionConversionFactor(ElevatorConstants.inchPerRotation/ElevatorConstants.gearing)
        .velocityConversionFactor(ElevatorConstants.inchPerRotation/ElevatorConstants.gearing/60);
    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,0.0);
        
        // .maxMotion.maxVelocity(ElevatorConstants.maxVelocity)
        //     .maxAcceleration(ElevatorConstants.maxAcceleration);
    leftConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ElevatorConstants.LevelMAX)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0);
    
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftConfig.follow(LeftMotor, true)
    .inverted(false);
    RightMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftMotor.getEncoder().setPosition(0);
    //RightMotor.getEncoder().setPosition(0);

    new Trigger(() -> !lowerLimit.get())
    .onTrue(runOnce(() -> {
        LeftMotor.stopMotor();
        //RightMotor.stopMotor();
        LeftMotor.getEncoder().setPosition(0);
        //RightMotor.getEncoder().setPosition(0);
    }));
} //do setup here, use the gear ratios/

public void goToHeight (double height) {
    SmartDashboard.putNumber("height request", height);
leftPID.setReference(height,ControlType.kPosition,
    ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("Height requested", height);
//rightPID.setReference(height,ControlType.kMAXMotionPositionControl,
 //   ClosedLoopSlot.kSlot0, ElevatorConstants.gravityCompensator, ArbFFUnits.kVoltage);
}

public void drive(double pwr) {
    LeftMotor.set(pwr);
    //RightMotor.set(pwr);
}

public double getHeight () {
    return LeftMotor.getEncoder().getPosition();
}

@Override
public void periodic() {
    SmartDashboard.putBoolean("elev bottom", lowerLimit.get());
    SmartDashboard.putBoolean("elev top", upperLimit.refresh().getValueAsDouble() != 0);
    SmartDashboard.putNumber("elev height", absEncoder.get());
}
}

