package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.Configs.SparkConfigs;

public class SwerveModule extends SubsystemBase{
    private final SparkMax azimuth;
    private final TalonFX drive;
    private final AbsoluteEncoder azimuthEncoder;
    private final SparkClosedLoopController azimuthPID;
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final VelocityVoltage request = new VelocityVoltage(0.0).withSlot(0);

    /**
     * Create a new FRC 1706 NEOKrakenSwerveModule Object
     *
     * @param moduleID  module ID also CAN ID for Azimuth and Drive MCs.
     * @param offset  The offset for the analog encoder.
     */  
    public SwerveModule(int driveID, int azimuthID, double offset)  {
        configurePID();
        drive = new TalonFX(driveID);
        drive.getConfigurator().apply(slot0Configs);
        drive.getConfigurator()
            .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true));
        drive.setNeutralMode(NeutralModeValue.Brake);

        azimuth = new SparkMax(azimuthID, MotorType.kBrushless);
        azimuth.configure(SparkConfigs.getAzimuthConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        azimuthEncoder = azimuth.getAbsoluteEncoder();

        azimuthPID = azimuth.getClosedLoopController();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getStateAngle()));
    }

    public void configurePID() {
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.05; // An error of 1 rps results in 0.05 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    public double getDriveVelocity() {
        return drive.getVelocity().getValueAsDouble() * ModuleConstants.kToMeters;
    }

    public double getDrivePosition() {
        return drive.getPosition().getValueAsDouble() * ModuleConstants.kToMeters;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(new Rotation2d(getStateAngle()));
        SwerveModuleState state = desiredState;
        double metersToRotations = state.speedMetersPerSecond * (1 / ModuleConstants.kToMeters);
        drive.setControl(request.withVelocity(metersToRotations).withSlot(0));
        azimuthPID.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public double getStateAngle() {
        return azimuthEncoder.getPosition();
    }

    public void stop() {
        drive.stopMotor();
        azimuth.stopMotor();
    }
}
