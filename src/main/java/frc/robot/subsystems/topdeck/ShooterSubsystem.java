package frc.robot.subsystems.topdeck;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry targetVelocityEntry = tab
            .add("Target Velocity RPM", TARGET_VELOCITY_RPM)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    GenericEntry motor1VelocityEntry = tab
            .add("Motor 1 Velocity", 1)
            .getEntry();
    GenericEntry motor2VelocityEntry = tab
            .add("Motor 2 Velocity", 0)
            .getEntry();

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final TalonFX shooterMotor;
   private final TalonFX shooterMotor2;
    // private final RelativeEncoder encoder;
    // private final RelativeEncoder encoder2;
    private PIDController pidController;
    private PIDController pidController2;

    // PID Constants - tune these!
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ShooterSubsystem() {
        shooterMotor = new TalonFX(SHOOTER_MOTOR_ID_RIGHT);
       shooterMotor2 = new TalonFX(SHOOTER_MOTOR_ID_LEFT);
        // encoder = shooterMotor.getEncoder();
        // encoder2 = shooterMotor2.getEncoder();

        pidController = new PIDController(kP, kI, kD);
        pidController2 = new PIDController(kP, kI, kD);

        configureMotor();
    }

    private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfiguration config2 = new TalonFXConfiguration();

    config2.CurrentLimits.StatorCurrentLimit = 80;
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;

config.CurrentLimits.StatorCurrentLimit = 80; // default is often 40A
config.CurrentLimits.StatorCurrentLimitEnable = true;
            StatusCode status = shooterMotor.getConfigurator().apply(config);
            System.out.println("Motor config status: " + status);

            StatusCode status2 = shooterMotor2.getConfigurator().apply(config2);
            System.out.println("Motor 2 config status: " + status2);
        // Tune these gains for your mechanism
//        Slot0Configs slot0 = config.Slot0;
        Slot0Configs slot01 = new Slot0Configs();
        Slot0Configs slot0 = new Slot0Configs();
/*
        slot0.kP = 2.4;   // proportional
        slot0.kI = 0.0;   // integral
        slot0.kD = 0.1;   // derivative
        slot0.kS = 0.25;  // static friction feedforward (volts)
        slot0.kV = 0.12;  // velocity feedforward (volts per rot/s)
*/        
        slot01.kS = 0.099609375;   
        slot01.kV = 0.12800000607967377; // 12 / 94
        slot01.kP = 20;   
        slot01.kA = 0.50;
        slot01.kI = 0.0;
        slot01.kD = 0.0;

        slot0.kS = 0.099609375;   
        slot0.kV = 0.12800000607967377; // 12 / 94
        slot0.kP = 20;   
        slot0.kA = 0.50;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        // Optional: set motion magic / current limits / etc.
        // config.MotionMagic.MotionMagicCruiseVelocity = 80;
        // config.MotionMagic.MotionMagicAcceleration = 160;

            status = shooterMotor.getConfigurator().apply(slot01);
            System.out.println("Motor 1 slot 0 config status: " + status);
            Slot0Configs verify = new Slot0Configs();
            shooterMotor.getConfigurator().refresh(verify);
            System.out.println("kP: " + verify.kP);
            System.out.println("kV: " + verify.kV);

            status2 = shooterMotor2.getConfigurator().apply(slot0);
            System.out.println("Motor 2 slot 0 config status: " + status2);
             Slot0Configs verify2 = new Slot0Configs();
             shooterMotor2.getConfigurator().refresh(verify2);
             System.out.println("kP: " + verify2.kP);
             System.out.println("kV: " + verify2.kV);
    }

    /**
     * Spins the shooter to the target velocity
     */
    public void runShooter() {
        runShooterAtVelocity(targetVelocityEntry.getDouble(70));
    }

    /**
     * Spins the shooter at a custom velocity
     * 
     * @param velocityRPM Target velocity in rotations per minute (RPM)
     */
    public boolean runShooterAtVelocity(double velocityRPM) {
        // Calculate PID output for each motor
        // double output1 = pidController.calculate(encoder.getVelocity(), velocityRPM);
        // double output2 = pidController2.calculate(encoder2.getVelocity(), velocityRPM);
        
        // Set motor outputs
        // shooterMotor.set(output1);
        // shooterMotor2.set(output2);
        // shooterMotor.set(0.73);
        
        //shooterMotor.setControl(velocityRequest.withVelocity(velocityRPM));
        shooterMotor.setControl(new VelocityVoltage(velocityRPM));
        shooterMotor2.setControl(new VelocityVoltage(velocityRPM));

        return shooterMotor.getVelocity().getValueAsDouble() >= velocityRPM - 5 && shooterMotor.getVelocity().getValueAsDouble() <= velocityRPM + 5; /*&& shooterMotor2.getVelocity().getValueAsDouble() >= velocityRPM - 5 && shooterMotor2.getVelocity().getValueAsDouble() <= velocityRPM + 5;*/ // Check if within 5 RPM of target

    }

    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        shooterMotor.stopMotor();
       shooterMotor2.stopMotor();
    }

    public void spinUpShooter() {
        shooterMotor2.setControl(new VelocityVoltage(30));
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return Current velocity in rotations per minute (RPM)
     */
    public double getVelocity() {
        //return (shooterMotor.getRotorVelocity().getValueAsDouble() + shooterMotor2.getRotorVelocity().getValueAsDouble()) / 2.0;
        return (shooterMotor.getRotorVelocity().getValueAsDouble());
    }

    // /**
    //  * Checks if the shooter is at target velocity
    //  * 
    //  * @return true if within tolerance
    //  */
    // public boolean atTargetVelocity() {
    //     double tolerance = 100.0; // RPM tolerance
    //     return (Math.abs(getVelocity() - TARGET_VELOCITY_RPM) < tolerance);
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter/Velocity RPM", getVelocity());
        // motor1VelocityEntry.setDouble(encoder.getVelocity());
        // motor2VelocityEntry.setDouble(encoder2.getVelocity());
        SmartDashboard.putNumber("Shooter/Target RPM", TARGET_VELOCITY_RPM);
        // SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putNumber("Motor Position", shooterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Velocity", shooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Motor VOLTAGE", shooterMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Closed Loop Reference", shooterMotor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Closed Loop Output", shooterMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Motor Voltage", shooterMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putBoolean("Has Fault", shooterMotor.getFaultField().getValueAsDouble() != 0);
        SmartDashboard.putNumber("Fault Field", shooterMotor.getFaultField().getValueAsDouble());
        SmartDashboard.putString("Control Mode", shooterMotor.getControlMode().toString());
        SmartDashboard.putBoolean("Is Alive", shooterMotor.isAlive());   }
}