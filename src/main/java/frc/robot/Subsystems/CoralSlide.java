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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SlideConstants;

public class CoralSlide extends SubsystemBase {
    private final SparkMax motor = new SparkMax(SlideConstants.motorID, MotorType.kBrushless);
    private final SparkClosedLoopController ctrl = motor.getClosedLoopController();

    public CoralSlide() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .inverted(false)
        .encoder.positionConversionFactor(1/SlideConstants.gearRatio/SlideConstants.screwRatio);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(SlideConstants.kP);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void goToPosition(double pos) {
        ctrl.setReference(pos, ControlType.kPosition);
    }

    public Command goLeft() {
        return runOnce(() -> goToPosition(SlideConstants.leftPos));
    }
    
    public Command goCenter() {
        return runOnce(() -> goToPosition(SlideConstants.centerPos));
    }
    
    public Command goRight() {
        return runOnce(() -> goToPosition(SlideConstants.rightPos));
    }
    
}
