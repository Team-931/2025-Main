// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkClosedLoopController leftPID;
  private final SparkClosedLoopController rightPID;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    leftMotor = new SparkMax(AlgaeConstants.leftMotorID, MotorType.kBrushless);
    rightMotor = new SparkMax(AlgaeConstants.rightMotorID, MotorType.kBrushless);

     leftPID = leftMotor.getClosedLoopController();
     rightPID = rightMotor.getClosedLoopController();
    

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();


    leftConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .inverted(false);
    leftConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(AlgaeConstants.kP, AlgaeConstants.kI, AlgaeConstants.kD);

    rightConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .inverted(true);
    rightConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(AlgaeConstants.kP, AlgaeConstants.kI, AlgaeConstants.kD);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }

  public Command in(){
    return runOnce(()-> {
      leftPID.setReference(AlgaeConstants.motorVelocity, ControlType.kVoltage);
       rightPID.setReference(AlgaeConstants.motorVelocity, ControlType.kVoltage);
      });

  }

  public Command out() {
    return runOnce(()-> {
      leftPID.setReference(-AlgaeConstants.motorVelocity, ControlType.kVoltage);
       rightPID.setReference(-AlgaeConstants.motorVelocity, ControlType.kVoltage);
      }) ;
  }

  public Command stop() {
      return runOnce(()-> {
        leftPID.setReference(0, ControlType.kVoltage); //Can also use stopMotor()
         rightPID.setReference(0, ControlType.kVoltage);
        });
    
  }

  public Trigger caughtOneTrigger = new Trigger(isAlgae());
  public void stopOnAlgaeBinding() {
    caughtOneTrigger.onTrue(stop());
  }

  private Timer currentTimer = new Timer();

  public BooleanSupplier isAlgae(){
    return ()-> {
      if (leftMotor.getOutputCurrent() >= AlgaeConstants.highCurrent){
        currentTimer.start(); // no effect if is running
        return (currentTimer.hasElapsed(AlgaeConstants.highCurrentTime));
      }
      else{
        currentTimer.reset();
        currentTimer.stop();
        return false;
      }

    }; //Come back and edit this later
  }

  public SparkAbsoluteEncoder getWristEncoder() {
    return leftMotor.getAbsoluteEncoder();
  }
}
