// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final TalonFX drivingTalon;
  private final SparkMax m_turningSparkMax;

  private final StatusSignal<AngularVelocity> m_drivingVelocity;
  private final StatusSignal<Angle> m_drivingPosition;
  private final AbsoluteEncoder m_turningEncoder;

  
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  //private final VelocityDutyCycle talonCycle;
  private final VelocityVoltage  talonCycleVoltage;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivingTalon = new TalonFX(drivingCANId);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    
    //m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.



    m_drivingVelocity = drivingTalon.getVelocity();
    m_drivingPosition = drivingTalon.getPosition();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();  
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput)
    .p(ModuleConstants.kTurningP)
    .i(ModuleConstants.kTurningI)
    .d(ModuleConstants.kTurningD)
    .outputRange(ModuleConstants.kTurningMinOutput,
    ModuleConstants.kTurningMaxOutput);
    //m_turningPIDController.setFeedbackDevice(m_turningEncoder);


    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
   
    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    config.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    .inverted(ModuleConstants.kTurningEncoderInverted);
   // m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
   // m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    //m_turningPIDController.setPositionPIDWrappingEnabled(true);
    //m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    //m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
  
    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    //m_turningPIDController.setP(ModuleConstants.kTurningP);
    //m_turningPIDController.setI(ModuleConstants.kTurningI);
    //m_turningPIDController.setD(ModuleConstants.kTurningD);
    //m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    /*m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);*/

    config.idleMode(ModuleConstants.kTurningMotorIdleMode)
    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit) ;
    //m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
   //not limiting drive current
    //m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

    driveConfiguration.Slot0.kP = ModuleConstants.kDrivingP;
    driveConfiguration.Slot0.kI = ModuleConstants.kDrivingI;
    driveConfiguration.Slot0.kD = ModuleConstants.kDrivingD;
    driveConfiguration.Slot0.kV = ModuleConstants.kDrivingFF;

    driveConfiguration.Slot1.kP = 12 /*volts*/ * ModuleConstants.kDrivingP;
    driveConfiguration.Slot1.kI = 12 /*volts*/ * ModuleConstants.kDrivingI;
    driveConfiguration.Slot1.kD = 12 /*volts*/ * ModuleConstants.kDrivingD;
    driveConfiguration.Slot1.kV = 12 /*volts*/ * ModuleConstants.kDrivingFF;

    //set idle
    driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // if (drivingCANId == 8 || drivingCANId == 4) {
    //   driveConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // }



    driveConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;

    //talonCycle = new VelocityDutyCycle(0);
    
    talonCycleVoltage = new VelocityVoltage(0).withSlot(1);


    //drivingTalon.burnFlash();
    drivingTalon.getConfigurator().apply(driveConfiguration);
    m_turningSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    drivingTalon.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingVelocity.getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingPosition.getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void  setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = correctedDesiredState;//SwerveModuleState.optimize(correctedDesiredState,
       // new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingTalon//.set(optimizedDesiredState.speedMetersPerSecond);
    //.setControl(talonCycle .withVelocity(optimizedDesiredState.speedMetersPerSecond) );
    .setControl(talonCycleVoltage .withVelocity(optimizedDesiredState.speedMetersPerSecond) );
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingTalon.setPosition(0);
  }
}
