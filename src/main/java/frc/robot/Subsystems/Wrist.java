package frc.robot.Subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    // add profiled PID and ArmFeedForward. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#feedforward-control-in-wpilib
    private final SparkMax wristMotor;

    private final SparkAbsoluteEncoder wristEncoder;

    private final SparkClosedLoopController wristPID;



    public Wrist(SparkAbsoluteEncoder sparkAbsoluteEncoder) {
    wristMotor = new SparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristEncoder = sparkAbsoluteEncoder;

    wristPID = wristMotor.getClosedLoopController();

    SparkMaxConfig wristConfig = new SparkMaxConfig();

    
    wristConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .inverted(false)
    .absoluteEncoder.positionConversionFactor(0);
    wristConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void goToWristPosition (double pos) {
        wristPID.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot1, WristConstants.gravityCompensation, ArbFFUnits.kVoltage);
    }

    private int delay;
    private static final int delayCt = 5;
    // reset encoders
    @Override
    public void periodic() {
        if(delay > 0) {
            --delay;
            return;
        }
        delay = delayCt;
        double curPos = wristEncoder.getPosition();
        if (Math.abs(wristMotor.getEncoder().getPosition() - curPos)
             > WristConstants.posTolerance &&
            Math.abs(wristEncoder.getVelocity()) < WristConstants.velTolerance)
            {
               wristMotor.getEncoder().setPosition(curPos);
            }
    }

    public void setWristPosition(String pos) {
        if (pos.equals("collectCoral")) goToWristPosition(WristConstants.coralCollectionPosition);
        if (pos.equals("bargePos")) goToWristPosition(WristConstants.bargePosition);
        if (pos.equals("L4Pos")) goToWristPosition(WristConstants.L4Position);
        if (pos.equals("L23Pos")) goToWristPosition(WristConstants.L23Position);
        if (pos.equals("algaeintake")) goToWristPosition(WristConstants.algaeIntake);
    }

    public double getPosition() {
        return wristEncoder.getPosition();
    }

}
