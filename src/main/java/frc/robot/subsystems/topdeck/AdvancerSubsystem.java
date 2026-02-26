package frc.robot.subsystems.topdeck;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AdvancerConstants.*;

public class AdvancerSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry maxspeedEntry = tab
            .add("Max Speed Advancer", 1.0)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .getEntry();
    private final SparkMax AdavancerMotor;
    private final SparkMax AdvancerRoller;

    public AdvancerSubsystem() {
        AdavancerMotor = new SparkMax(ADVANCER_MOTOR_ID, MotorType.kBrushless);
        AdvancerRoller = new SparkMax(ADVANCER_ROLLER_ID, MotorType.kBrushless);
        SparkBaseConfig AdvancerMotorConfig = new SparkMaxConfig();

        AdvancerMotorConfig.smartCurrentLimit(40, 40);
        AdvancerMotorConfig.disableFollowerMode();

        AdvancerMotorConfig.idleMode(IdleMode.kBrake);

        AdvancerMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
        AdvancerMotorConfig.signals.primaryEncoderPositionPeriodMs(5);

        AdavancerMotor.configure(AdvancerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkBaseConfig AdvancerRollerConfig = new SparkMaxConfig();
        AdvancerRollerConfig.smartCurrentLimit(10, 10);
        AdvancerRollerConfig.disableFollowerMode();

        AdvancerRollerConfig.idleMode(IdleMode.kBrake);

        AdvancerRollerConfig.signals.primaryEncoderPositionAlwaysOn(true);
        AdvancerRollerConfig.signals.primaryEncoderPositionPeriodMs(5);

        AdvancerRoller.configure(AdvancerRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reverse() {
        AdavancerMotor.set(ADVANCER_SPEED);
        // AdvancerRoller.set(1);
    }

    public void stopAdvancer() {
        AdavancerMotor.set(0);
        AdvancerRoller.set(0);
    }

    public void climber(double joystick) {
        AdvancerRoller.set(joystick);
    }

    public void advance() {
        AdavancerMotor.set(-ADVANCER_SPEED);
        // AdvancerRoller.set(-1);
    }
}
