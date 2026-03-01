// package frc.robot.subsystems.topdeck;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import static frc.robot.Constants.HoodConstants.*;

// public class HoodSubsystem extends SubsystemBase{
//     private final SparkMax hoodMotor;
//     private final RelativeEncoder hoodEncoder;
//     private double targetRotations = HOOD_HOME_ROTATIONS;

//     public HoodSubsystem() {
//         hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
//         hoodEncoder = hoodMotor.getEncoder();
//         configureMotor();
//     }

//     private void configureMotor() {
//         SparkMaxConfig config = new SparkMaxConfig();
//         config.disableFollowerMode();
//         config.idleMode(IdleMode.kBrake);
//         config.smartCurrentLimit(20, 20);
//         config.inverted(HOOD_INVERTED);
//         config.voltageCompensation(12.0);
//         hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     public void adjustForDistance(double distanceMeters) {
//         double clampedDistance = clamp(distanceMeters, HOOD_MIN_DISTANCE_METERS, HOOD_MAX_DISTANCE_METERS);
//         double distanceSpan = HOOD_MAX_DISTANCE_METERS - HOOD_MIN_DISTANCE_METERS;
//         double percent = distanceSpan <= 0.0 ? 0.0 : (clampedDistance - HOOD_MIN_DISTANCE_METERS) / distanceSpan;
//         double target = HOOD_MIN_ROTATIONS + percent * (HOOD_MAX_ROTATIONS - HOOD_MIN_ROTATIONS);
//         setTargetPosition(target);
//     }

//     public void setTargetPosition(double rotations) {
//         targetRotations = clamp(rotations, HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS);
//         holdTargetPosition();
//     }

//     public void stop() {
//         hoodMotor.stopMotor();
//     }

//     public double getPositionRotations() {
//         return hoodEncoder.getPosition();
//     }

//     public double getTargetRotations() {
//         return targetRotations;
//     }

//     private void holdTargetPosition() {
//         double error = targetRotations - hoodEncoder.getPosition();
//         if (Math.abs(error) <= HOOD_POSITION_TOLERANCE_ROTATIONS) {
//             hoodMotor.set(0.0);
//             return;
//         }

//         double output = clamp(error * HOOD_KP, -HOOD_MAX_OUTPUT, HOOD_MAX_OUTPUT);
//         hoodMotor.set(output);
//     }

//     private double clamp(double value, double min, double max) {
//         return Math.max(min, Math.min(max, value));
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Hood/PositionRotations", getPositionRotations());
//         SmartDashboard.putNumber("Hood/TargetRotations", getTargetRotations());
//     }

// }
