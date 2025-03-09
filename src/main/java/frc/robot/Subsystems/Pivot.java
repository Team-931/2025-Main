package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Pivot extends SubsystemBase {
    // add profiled PID and ArmFeedForward. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#feedforward-control-in-wpilib
    private final SparkMax wristMotor;

    private final SparkClosedLoopController wristPID;

    public Pivot() {
    wristMotor = new SparkMax(WristConstants.wristMotorID, MotorType.kBrushless);
    wristPID = wristMotor.getClosedLoopController();

    SparkMaxConfig wristConfig = new SparkMaxConfig();
    
    wristConfig 
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .inverted(false);
    wristConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(WristConstants.kP, WristConstants.kI, WristConstants.kD);

    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //add code for the 4 positions
    //The floor position well need to be different from the L1, 2, and 3 positions
    

}
