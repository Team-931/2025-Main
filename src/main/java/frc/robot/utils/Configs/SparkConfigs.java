package frc.robot.utils.Configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public class SparkConfigs {

    // swerve module azimuth
    public static SparkMaxConfig getAzimuthConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
        .voltageCompensation(12.6)
        .idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput)
        .p(ModuleConstants.kTurningP)
        .i(ModuleConstants.kTurningI)
        .d(ModuleConstants.kTurningD)
        .outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);
        config.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
        .inverted(ModuleConstants.kTurningEncoderInverted);
        return config;
    }
}
