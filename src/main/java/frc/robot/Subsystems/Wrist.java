package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    // add profiled PID and ArmFeedForward. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#feedforward-control-in-wpilib
    // private final SparkMax wristMotor;
    // private final AbsoluteEncoder wristEncoder;
    //private final SparkClosedLoopController wristPID;

    //change motor type
    private final TalonFX drivingTalon;
    private MotionMagicVoltage driveControlRequest = new MotionMagicVoltage(WristConstants.initialPosition);

    private SparkLimitSwitch limitSwitch;

    private final StatusSignal<AngularVelocity> m_drivingVelocity;
    private final StatusSignal<Angle> m_drivingPosition;
    
  
   
    public Wrist(SparkLimitSwitch sparkLimitSwitch) {
        drivingTalon = new TalonFX(WristConstants.wristMotorID);

        limitSwitch = sparkLimitSwitch;

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    
    //m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.



    m_drivingVelocity = drivingTalon.getVelocity();
    m_drivingPosition = drivingTalon.getPosition();
    
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();


    //Problems are stemming from this origionally being a SparkMax config, and not a talon config. This needs to be edited so that it would
    //correctly configure the talon motor.
    //wristConfig 
    // .setNeutralMode(NeutralMode.Brake)
    // .smartCurrentLimit(20)
    // .inverted(false)
    //.absoluteEncoder.positionConversionFactor(0);
    //wristConfig.ClosedLoop
    //.feedbackSensor()
    //.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //.pid(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    // drivingTalon.getConfigurator(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristConfig.MotorOutput.Inverted =  InvertedValue.CounterClockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 70;
    wristConfig.Feedback.SensorToMechanismRatio = WristConstants.gearRatio;//fix this value
    
    wristConfig.SoftwareLimitSwitch
    .withForwardSoftLimitEnable(true)
    .withReverseSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(WristConstants.L4Position)
    .withReverseSoftLimitThreshold(WristConstants.coralCollectionPosition);

    wristConfig.Slot0
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(WristConstants.gravityCompensation)
        .withKP(WristConstants.kP)
        .withKI(WristConstants.kI) 
        .withKV(WristConstants.kV);//slot0 method requires individual changing of kp, ki, and kd

    wristConfig.MotionMagic
        .withMotionMagicCruiseVelocity(WristConstants.maxVel)
        .withMotionMagicAcceleration(WristConstants.maxAccel);

    drivingTalon.getConfigurator().apply(wristConfig);
    drivingTalon.setPosition(WristConstants.initialPosition);
    }

    public void goToWristPosition (double pos) {
        drivingTalon.setControl(driveControlRequest.withPosition(pos).withLimitReverseMotion(limitSwitch.isPressed()));
        //wristPID.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot1, WristConstants.gravityCompensation, ArbFFUnits.kVoltage);
    }

    public void setWristPosition(String pos) {
        if (pos.equals("collectCoral")) goToWristPosition(WristConstants.coralCollectionPosition);
        if (pos.equals("bargePos")) goToWristPosition(WristConstants.bargePosition);
        if (pos.equals("L4Pos")) goToWristPosition(WristConstants.L4Position);
        if (pos.equals("L23Pos")) goToWristPosition(WristConstants.L23Position);
        if (pos.equals("algaeintake")) goToWristPosition(WristConstants.algaeIntake);
    }

    public double getPosition() {
        return m_drivingPosition.refresh().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if(limitSwitch.isPressed()) 
            drivingTalon.setControl(driveControlRequest.withLimitReverseMotion(true));
    }

}
