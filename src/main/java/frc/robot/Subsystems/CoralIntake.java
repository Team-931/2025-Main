package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    .inverted(true)
    .limitSwitch
      .forwardLimitSwitchEnabled(false)
      .reverseLimitSwitchEnabled(false);
    coralConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD);

    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command in() {
    return runOnce(()-> {
      coralPID.setReference(CoralConstants.motorVelocity, ControlType.kVoltage);
      }) ;
  }

  public Command out() {
    return runOnce(()-> {
      coralPID.setReference(-CoralConstants.motorVelocity, ControlType.kVoltage);
      }) ;
  }

  public Command stop() {
    return runOnce(()-> {
      coralPID.setReference(0, ControlType.kVoltage);
      }) ;
  }

  public Trigger caughtOneTrigger = new Trigger(isCoral());

  public void stopOnCoralBinding() {
    caughtOneTrigger.onTrue(stop());
  }

  private Timer currentTimer = new Timer();

  public BooleanSupplier isCoral(){
    return ()-> {
      if (coralMotor.getOutputCurrent() >= CoralConstants.highCurrent){
        currentTimer.start(); // no effect if is running
        return (currentTimer.hasElapsed(CoralConstants.highCurrentTime));
      }
      else{
        currentTimer.reset();
        currentTimer.stop();
        return false;
      }
    };
  }

  public Trigger slideRightLimit() {
    return new Trigger(() -> coralMotor.getForwardLimitSwitch().isPressed());
  }

  public Trigger slideLeftLimit() {
    return new Trigger(() -> coralMotor.getReverseLimitSwitch().isPressed());
  }

  public Trigger slideCentered() {
    return new Trigger(() -> coralMotor.getAnalog().getPosition() < 1.5 /* V */);
  }

}
