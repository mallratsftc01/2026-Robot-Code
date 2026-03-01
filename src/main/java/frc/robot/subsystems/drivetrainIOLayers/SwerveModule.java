package frc.robot.subsystems.drivetrainIOLayers;

import com.revrobotics.spark.SparkMax;

import static frc.robot.Constants.SwerveConstants.MOTOR_MAX_RPM;

import java.io.FileWriter;
import java.io.IOException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.SwerveModuleConstants;
import frc.lib.util.SparkConfigUtils;
import frc.robot.Constants.SwerveConstants;
import frc.robot.model.MetricName;
import frc.robot.service.MetricService;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule extends SubsystemBase {

    // private ShuffleboardTab swerve_tab = Shuffleboard.getTab("DriveTrain");

    // private GenericEntry m_driveMotor_set_entry = swerve_tab.add("m_driveMotor
    // set", 0).getEntry();
    // private GenericEntry m_driveMotor_actual_entry = swerve_tab.add("m_driveMotor
    // actual", 0).getEntry();
    // private GenericEntry m_turningMotor_set_entry =
    // swerve_tab.add("m_turningMotor set", 0).getEntry();
    // private GenericEntry m_turningMotor_actual_entry =
    // swerve_tab.add("m_turningMotor actual", 0).getEntry();
    // private GenericEntry m_driveMotor_velocity_entry =
    // swerve_tab.add("m_driveMotor velocity", 0).getEntry();
    // private GenericEntry m_driveMotor_get_entry = swerve_tab.add("m_driveMotor
    // get", 0).getEntry();
    // private GenericEntry m_drive_encoder_entry = swerve_tab.add("drive encoder",
    // 0).getEntry();
    // private GenericEntry m_turn_encoder_entry = swerve_tab.add("turn encoder",
    // 0).getEntry();
    // private GenericEntry target_entry = swerve_tab.add("target", 0).getEntry();
    // private GenericEntry turnOutput_entry = swerve_tab.add("turnOutput",
    // 0).getEntry();

    private static final double DISTANCE_CORRECTION_FACTOR = 0.97;

    private SimpleMotorFeedforward m_driveFeedforward;

    private static final double TURN_KS = 0.1; // volts, tune this


    private PIDController turningPidController = new PIDController(2.9, 0.1, 0);
    // new ProfiledPIDController(
    // 3.1, // Proportional gain
    // 0.0, // Integral gain
    // 0.01,
    // new TrapezoidProfile.Constraints(
    // SwerveConstants.kModuleMaxAngularVelocity,
    // SwerveConstants.kModuleMaxAngularAcceleration));
    // KB not being used right now might go back into use later
    // private ProfiledPIDController drivingPidController = new
    // ProfiledPIDController(
    // 1.0, // Proportional gain
    // 0.0, // Integral gain
    // 0.0,
    // new TrapezoidProfile.Constraints(
    // SwerveConstants.MaxMetersPersecond,
    // SwerveConstants.kMaxAceceration));

    private final PIDController m_drivePIDController;

    private SparkMax driveMotor;
    private SparkMax angleMotor;

    private CANcoder Encoder;
    private RelativeEncoder m_driveEncoder;

    private double angleOffset;

    private int moduleNumber;

    // ShuffleboardLayout module_layout = swerve_tab
    // .getLayout("Elevator", BuiltInLayouts.kGrid)
    // .withSize(2, 6)
    // .withPosition(0, moduleNumber)
    // .withProperties(Map.of("Label position", "HIDDEN"));

    private static final boolean STUCK_PROTECTION_ENABLED = true;
    private static final long ANGLE_STUCK_TIME_THRESHOLD_MS = 300;
    private static final long ANGLE_CORRECTION_TIME_THRESHOLD_MS = 50;
    private static final double ANGLE_DIVERGENCE_TOLERANCE = 7.0 * Math.PI / 180.0; // radians
    private long angleDivergenceStartTime = -1;
    private long angleCorrectionStartTime = -1;

    @SuppressWarnings("removal")
    public SwerveModule(SwerveModuleConstants s) {

        driveMotor = new SparkMax(s.driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(s.angleMotorID, MotorType.kBrushless);
        SparkBaseConfig driveMotorConfig = new SparkMaxConfig();
        SparkBaseConfig angleMotorConfig = new SparkMaxConfig();
        driveMotorConfig.smartCurrentLimit(40, 40);
        driveMotorConfig.disableFollowerMode();

        // Set inversion based on module number
        // Modules 0 and 3 are NOT inverted, modules 1 and 2 ARE inverted
        if (s.moduleNumber == 0 || s.moduleNumber == 3) {
            driveMotorConfig.inverted(false);
        } else {
            driveMotorConfig.inverted(true);
        }

        driveMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.apply(driveMotorConfig);

        angleMotorConfig.inverted(true);

        driveMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
        driveMotorConfig.signals.primaryEncoderPositionPeriodMs(5);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Encoder = new CANcoder(s.cancoderID);

        CANcoderConfiguration cc_config = new CANcoderConfiguration();
        cc_config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_config.MagnetSensor.MagnetOffset = s.angleOffset;
        Encoder.getConfigurator().apply(cc_config);

        m_driveEncoder = driveMotor.getEncoder();

        angleOffset = 0;

        this.moduleNumber = s.moduleNumber;

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        m_driveEncoder.setPosition(0);

        captureConfig();

        m_drivePIDController = new PIDController(s.pidP, s.pidI, 0);

        m_driveFeedforward = new SimpleMotorFeedforward(0.1, 2.4, 0.0); // kS, kV, kA

        m_drivePIDController.setIZone(0.06);
        m_drivePIDController.setIntegratorRange(-0.06, 0.06);

        turningPidController.setIZone(Math.PI / 10);
        turningPidController.setIntegratorRange(-Math.PI / 20, Math.PI / 20);
    }

    @Override
    public void periodic() {
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]encoder raw " + moduleNumber, getRawAngle());
        }
        if (RobotState.isEnabled()) {
            MetricService.publish(MetricName.driveEncoderValue(moduleNumber), m_driveEncoder.getPosition());
        }
    }

    private double encoderValue() {
        var retVal = getRawAngle();
        // SmartDashboard.putNumber("[Swerve]module " + moduleNumber, retVal);
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]encoder raw " + moduleNumber, retVal);
        }

        SmartDashboard.putNumber("[Swerve]encoder " + moduleNumber, (retVal * 1000) / 1000.0);
        SmartDashboard.putNumber("[Swerve]encoder degrees " + moduleNumber, (retVal * (180 / Math.PI) * 1000) / 1000.0);

        retVal = (retVal + angleOffset) % (2.0 * Math.PI); // apply offset for this encoder and map it back onto [0,
                                                           // 2pi]

        if (retVal < 0) {
            retVal += (2.0 * Math.PI); // map negative values to [0, 2pi]
        }

        // might need this so we're in the same range as the pid controller is
        // expecting.
        // retVal = retVal - Math.PI;
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("[Swerve]encoder adjusted " + moduleNumber, retVal);
        }

        SmartDashboard.putNumber("[Swerve]EncoderValue() " + moduleNumber, retVal);
        return (retVal);
    }

    private double getRawAngle() {
        // double retVal = Encoder.getvl() / RobotController.getVoltage5V(); // convert
        // voltage to %
        // retVal = 2.0 * Math.PI * retVal; // get % of circle encoder is reading
        // return retVal;
        return Encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
    var s = getConvertedVelocity();
    return new SwerveModuleState(
    s, new Rotation2d(encoderValue()));
    }

    private double getConvertedVelocity() {
        return (m_driveEncoder.getVelocity() / (60.0 * SwerveConstants.gearboxRatio))
                * ((SwerveConstants.kWheelRadius * 2) * Math.PI);
    }

    private double getCurrentSpeedAsPercentage() {
        return m_driveEncoder.getVelocity() / MOTOR_MAX_RPM;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // encode is % rotations
        var retVal = DISTANCE_CORRECTION_FACTOR
                * ((m_driveEncoder.getPosition() / SwerveConstants.gearboxRatio) * (SwerveConstants.kWheelRadius * 2)
                        * Math.PI); // distance

        // in
        // whatever
        // units
        // the
        // wheel
        // diameter
        // is
        // KB ^^^^ This is from 1 meter testing dont move/change
        return new SwerveModulePosition(retVal, new Rotation2d(encoderValue()));
    }

    public void SetDesiredState(SwerveModuleState desiredState) {
        double currentSpeedPercentage = getCurrentSpeedAsPercentage();
        double currentAngle = encoderValue();
        MetricService.publish(MetricName.currentAngle(moduleNumber), currentAngle);
        MetricService.publish(MetricName.actualVelocity(moduleNumber), currentSpeedPercentage);

        SmartDashboard.putNumber("Speed " + moduleNumber, currentSpeedPercentage);
        SmartDashboard.putNumber("[Swerve]Pre Optimize angle target degrees " + moduleNumber,
                desiredState.angle.getDegrees());
        SmartDashboard.putNumber("[Swerve]turn encoder" + moduleNumber, currentAngle);

        @SuppressWarnings("deprecation")
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(currentAngle));

        SmartDashboard.putNumber("[Swerve]After Optimize angle target degrees " + moduleNumber,
                state.angle.getDegrees());

        MetricService.publish(MetricName.reqSpeed(moduleNumber), state.speedMetersPerSecond);
        MetricService.publish(MetricName.reqTurn(moduleNumber), state.angle.getRadians());

        double currentDivergence = Math.abs(Rotation2d.fromRadians(state.angle.getRadians())
                .minus(Rotation2d.fromRadians(currentAngle)).getRadians());
        if (currentDivergence > ANGLE_DIVERGENCE_TOLERANCE && angleDivergenceStartTime == -1) {
            angleDivergenceStartTime = System.currentTimeMillis();
        } else if (currentDivergence <= ANGLE_DIVERGENCE_TOLERANCE) {
            angleDivergenceStartTime = -1;
            angleCorrectionStartTime = -1;
        }
        boolean stuck = angleDivergenceStartTime != -1
                && System.currentTimeMillis() - angleDivergenceStartTime > ANGLE_STUCK_TIME_THRESHOLD_MS;
        if (stuck && angleCorrectionStartTime == -1) {
            angleCorrectionStartTime = System.currentTimeMillis();
        }
        SmartDashboard.putBoolean("[Swerve] stuck detector " + moduleNumber, stuck);
        if (STUCK_PROTECTION_ENABLED && stuck) {
            state.angle = Rotation2d
                    .fromRadians(currentAngle + (currentAngle > state.angle.getRadians() ? 1.0 : -1.0) * Math.PI / 2.0);
            state.speedMetersPerSecond = 0.0;
            if (System.currentTimeMillis() - angleCorrectionStartTime > ANGLE_CORRECTION_TIME_THRESHOLD_MS) {
                angleDivergenceStartTime = -1;
                angleCorrectionStartTime = -1;
            }
        }

         final double turnOutput = turningPidController.calculate(currentAngle, state.angle.getRadians());

        double targetSpeedMetersPerSecond = state.speedMetersPerSecond
        * state.angle.minus(Rotation2d.fromRadians(currentAngle)).getCos();

double targetSpeedPercentage = targetSpeedMetersPerSecond / SwerveConstants.MaxMetersPersecond;

final double driveOutput = targetSpeedPercentage
        + m_drivePIDController.calculate(currentSpeedPercentage, targetSpeedPercentage)
        + m_driveFeedforward.calculate(targetSpeedMetersPerSecond) / 12.0;

        // No FlipSpeed needed - inversion is handled in motor configuration
        driveMotor.set(driveOutput);

        double turnFF = TURN_KS * Math.signum(turnOutput);
        angleMotor.set((turnOutput / Math.PI) + (turnFF / 12.0));


        MetricService.publish(MetricName.commandedSpeed(moduleNumber), driveOutput);
        MetricService.publish(MetricName.commandedTurn(moduleNumber), (turnOutput / Math.PI));

        // SmartDashboard.putNumber("[Swerve]m_driveMotor set " + moduleNumber,
        // driveOutput);
        // SmartDashboard.putNumber("[Swerve]m_turningMotor set " + moduleNumber,
        // turnOutput / SwerveConstants.kModuleMaxAngularVelocity);

        // SmartDashboard.putNumber("[Swerve]m_driveMotor actual" + moduleNumber,
        // getConvertedVelocity());
        // SmartDashboard.putNumber("[Swerve]m_driveMotor velocity" + moduleNumber,
        // currentSpeedPercentage);
        // SmartDashboard.putNumber("[Swerve]m_driveMotor get" + moduleNumber,
        // driveMotor.get());
        // SmartDashboard.putNumber("[Swerve]m_turningMotor actual" + moduleNumber,
        // angleMotor.get());

        // SmartDashboard.putNumber("[Swerve]drive encoder" + moduleNumber,
        // m_driveEncoder.getPosition());
        // SmartDashboard.putNumber("[Swerve]turn encoder" + moduleNumber,
        // currentAngle);

        // SmartDashboard.putNumber("[Swerve]turnOutput", turnOutput);
        // SmartDashboard.putNumber("[Swerve]target " + moduleNumber,
        // state.angle.getRadians());
    }

    public void SetDesiredStateWithFF(SwerveModuleState desiredState, double forceNewtons) {
    double currentSpeedPercentage = getCurrentSpeedAsPercentage();
    double currentAngle = encoderValue();

    @SuppressWarnings("deprecation")
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(currentAngle));

    double targetSpeedPercentage = (state.speedMetersPerSecond / SwerveConstants.MaxMetersPersecond)
            * state.angle.minus(Rotation2d.fromRadians(currentAngle)).getCos();

    // Convert force (N) -> acceleration (m/s²) -> feedforward voltage -> percent
    // F = ma, so a = F/mass. Approximation: use kA from feedforward
    double ffVolts = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    double ffPercent = ffVolts / 12.0;

    double pidOutput = m_drivePIDController.calculate(currentSpeedPercentage, targetSpeedPercentage);

    // Clamp drive output to ±1.0
    final double driveOutput = Math.max(-1.0, Math.min(1.0, 
            targetSpeedPercentage + pidOutput + ffPercent));

    // Clamp turn output to ±1.0
    double rawTurnOutput = turningPidController.calculate(currentAngle, state.angle.getRadians());
    final double turnOutput = Math.max(-1.0, Math.min(1.0, rawTurnOutput / Math.PI));

    driveMotor.set(driveOutput);
    angleMotor.set(turnOutput);
}

    private void captureConfig() {

        try {
            var logFileWriter = new FileWriter("/home/lvuser/sparkconfig_" + moduleNumber + ".txt", false);
            logFileWriter.write("Driving motor config for module " + moduleNumber + "\n");
            SparkConfigUtils.printConfig(driveMotor, logFileWriter);
            logFileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
