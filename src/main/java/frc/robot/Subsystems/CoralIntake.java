package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralIntake extends SubsystemBase {
    // similar controls as for Algae
    // implement bang-bang control for slide
    private final SparkMax coralMotor;
    
    private final SparkClosedLoopController coralPID;

  public CoralIntake() {
    coralMotor = new SparkMax(CoralConstants.coralMotorID, MotorType.kBrushless);
    coralPID = coralMotor.getClosedLoopController();

    SparkMaxConfig coralConfig = new SparkMaxConfig();

    coralConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .inverted(false);
    coralConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD);

    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command in() {
    return runOnce(()-> {
      coralPID.setReference(CoralConstants.motorVelocity, ControlType.kVelocity);
      }) ;
  }

  public Command out() {
    return runOnce(()-> {
      coralPID.setReference(-CoralConstants.motorVelocity, ControlType.kVelocity);
      }) ;
  }

  public Command stop() {
    return runOnce(()-> {
      coralPID.setReference(0, ControlType.kVelocity);
      }) ;
  }
}
