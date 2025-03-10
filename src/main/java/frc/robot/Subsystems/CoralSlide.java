package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SlideConstants;

public class CoralSlide extends SubsystemBase {
    private final SparkMax motor = new SparkMax(SlideConstants.motorID, MotorType.kBrushless);
    private final SparkClosedLoopController ctrl = motor.getClosedLoopController();
    private final DigitalInput left = new DigitalInput(SlideConstants.leftDIO);
    private final DigitalInput right = new DigitalInput(SlideConstants.rightDIO);
    private final DigitalInput centre = new DigitalInput(SlideConstants.centreDIO);

    // need config velocity PID
    public CoralSlide() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .inverted(true)
        .encoder.positionConversionFactor(1/SlideConstants.gearRatio/SlideConstants.screwRatio)
        .velocityConversionFactor(1/SlideConstants.gearRatio/SlideConstants.screwRatio);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SlideConstants.kP);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void goToPosition(double pos) {
        ctrl.setReference(pos, ControlType.kPosition);
    }


    // public Command goLeft() {
    //     return runOnce(() -> goToPosition(SlideConstants.leftPos));
    // }
    public Command goLeft() {
        return runOnce(()-> ctrl.setReference(SlideConstants.speed, ControlType.kVelocity)).until(isLeft());
    }
    
    // public Command goCenter() {
    //     return runOnce(() -> goToPosition(SlideConstants.centerPos));
    // }
    public Command goCenter() {
        if(motor.getEncoder().getPosition() > SlideConstants.centerPos) {
            return runOnce(()-> ctrl.setReference(SlideConstants.speed, ControlType.kVelocity)).until(isCentre());
        }
        else {
            return runOnce(()-> ctrl.setReference(-SlideConstants.speed, ControlType.kVelocity)).until(isCentre());
        }
    }
    
    // public Command goRight() {
    //     return runOnce(() -> goToPosition(SlideConstants.rightPos));
    // }
    public Command goRight() {
        return runOnce(()-> ctrl.setReference(-SlideConstants.speed, ControlType.kVelocity)).until(isRight());
    }

    private Trigger isLeft() {
        motor.getEncoder().setPosition(0);
        return new Trigger(()-> left.get());
    }

    private Trigger isCentre() {
        motor.getEncoder().setPosition(SlideConstants.centerPos);
        return new Trigger(()-> centre.get());
    }
    
    private Trigger isRight() {
        motor.getEncoder().setPosition(SlideConstants.rightPos);
        return new Trigger(()-> right.get());
    }
}
