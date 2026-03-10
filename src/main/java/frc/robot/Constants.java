// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.math.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class SwerveConstants {
                /*
                 * The locations for the modules must be relative to the center of the robot.
                 * Positive x values represent moving toward the front of the robot whereas
                 * positive y values represent moving toward the left of the robot.
                 */
                private static final double ROBOT_WIDTH = Units.inchesToMeters(24.0);
                private static final double ROBOT_LENGTH = Units.inchesToMeters(30.0);
                private static final double SWERVE_FROM_CORNER = Units.inchesToMeters(2.61);
                private static final double MODULE_OFFSET_X = ROBOT_WIDTH / 2 - SWERVE_FROM_CORNER; // 9.39 inches
                private static final double MODULE_OFFSET_Y = ROBOT_LENGTH / 2 - SWERVE_FROM_CORNER; // 12.39 inches

                public static final Translation2d m_backLeftLocation = new Translation2d(-MODULE_OFFSET_X,
                                MODULE_OFFSET_Y);
                public static final Translation2d m_backRightLocation = new Translation2d(-MODULE_OFFSET_X,
                                -MODULE_OFFSET_Y);
                public static final Translation2d m_frontRightLocation = new Translation2d(MODULE_OFFSET_X,
                                -MODULE_OFFSET_Y);
                public static final Translation2d m_frontLeftLocation = new Translation2d(MODULE_OFFSET_X,
                                MODULE_OFFSET_Y);

                public static final double MOTOR_MAX_RPM = 5676.0 * 1.05; // from testing motors are reaching ~105% max
                                                                          // rpm

                public static final double MaxMetersPersecond = 4.47;// 3.264903459; //4.47 This is calculated 5676rpm,
                                                                     // 4in
                                                                     // wheels, 6.75 gearbox
                public static final double kWheelRadius = 0.04648915887; // 0.0508;
                public static final double kModuleMaxAngularVelocity = 27.73816874; // This is calculated 5676rpm,
                                                                                    // 150/7:1
                                                                                    // gearbox in radians. 594.380 deg/s
                                                                                    // in
                                                                                    // pathplanner
                public static final double kModuleMaxAngularAcceleration = 18.85;// 4 * Math.PI; // radians per second
                                                                                 // squared
                public static final double gearboxRatio = 6.75;

                public static final double kMaxAceceration = 4.0;

                public static final class Mod1 {
                        public static final int driveMotorID = 11;
                        public static final int angleMotorID = 10;
                        public static final int canCoderID = 12;
                        public static final double angleOffset = -0.15185546875;
                        public static final double pidP = 2.3; // 2.2 prevous value
                        public static final double pidI = 0.75;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID, canCoderID, angleOffset, pidP, pidI, 0);
                }

                /** Front Right Module - Module 1 */
                public static final class Mod2 {
                        public static final int driveMotorID = 21;
                        public static final int angleMotorID = 20;
                        public static final int canCoderID = 22;
                        public static final double angleOffset = -0.07861328125;
                        public static final double pidP = 0.85; // 1.0 previous value
                        public static final double pidI = 0.2;
                        public static final double speedAdjustmentFactor = 1;// 1.798006206333298/1.891452461749773;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID, canCoderID, angleOffset, pidP, pidI, 1);
                }

                /** Back Left Module - Module 2 */
                public static final class Mod3 {
                        public static final int driveMotorID = 31;
                        public static final int angleMotorID = 30;
                        public static final int canCoderID = 32;
                        public static final double angleOffset = 0.21484375;
                        public static final double pidP = 2.3; // 1.8 previous value
                        public static final double pidI = 0.75;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID, canCoderID, angleOffset, pidP, pidI, 2);
                }

                /** Back Right Module - Module 3 */
                public static final class Mod4 {
                        public static final int driveMotorID = 41;
                        public static final int angleMotorID = 40;
                        public static final int canCoderID = 42;
                        public static final double angleOffset = 0.2578125;
                        public static final double pidP = 0.6; // 0.8 previous value
                        public static final double pidI = 0.3; // zach isn't helping me
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID, canCoderID, angleOffset, pidP, pidI, 3);
                }
        }

        public final static class JoystickConstants {
                public final static int DRIVER_USB = 0;
                public final static int OPERATOR_USB = 1;
                public final static int TEST_USB = 2;

                public final static int LEFT_Y_AXIS = 1;
                public final static int LEFT_X_AXIS = 0;
                public final static int RIGHT_X_AXIS = 4;
                public final static int RIGHT_Y_AXIS = 5;

                public final static int GREEN_BUTTON = 1;
                public final static int RED_BUTTON = 2;
                public final static int YELLOW_BUTTON = 4;
                public final static int BLUE_BUTTON = 3;

                public final static int LEFT_TRIGGER = 2;
                public final static int RIGHT_TRIGGER = 3;
                public final static int LEFT_BUMPER = 5;
                public final static int RIGHT_BUMPER = 6;

                public final static int BACK_BUTTON = 7;
                public final static int START_BUTTON = 8;

                public final static int POV_UP = 0;
                public final static int POV_RIGHT = 90;
                public final static int POV_DOWN = 180;
                public final static int POV_LEFT = 270;
        }

        public static final class Vision {
                public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2026RebuiltWelded
                                .loadAprilTagLayoutField();

                public static final Transform3d LeftCam = new Transform3d(Units.inchesToMeters(.75),
                                Units.inchesToMeters(15), Units.inchesToMeters(8.75),
                                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(90)));

                public static final Transform3d RightCam = new Transform3d(Units.inchesToMeters(0.75),
                                -Units.inchesToMeters(15), Units.inchesToMeters(8.75),
                                new Rotation3d(0, Math.PI / 4.0, Units.degreesToRadians(-90)));

                public static final Transform3d FrontCam = new Transform3d(Units.inchesToMeters(.75),
                                Units.inchesToMeters(15), Units.inchesToMeters(8.75),
                                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(90)));

                public static final Transform3d BackCam = new Transform3d(Units.inchesToMeters(0.75),
                                -Units.inchesToMeters(15), Units.inchesToMeters(8.75),
                                new Rotation3d(0, Math.PI / 4.0, Units.degreesToRadians(-90)));
                // The standard deviations of our vision estimated poses, which affect
                // correction rate
                // (Fake values. Experiment and determine estimation noise on an actual robot.)
                public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }

        public static final class ShooterConstants {
                // Constants
                public static final int SHOOTER_MOTOR_ID_RIGHT = 1;
                public static final int SHOOTER_MOTOR_ID_LEFT = 2;
                public static final double TARGET_VELOCITY_RPM = 700; 
                public static final double MOTOR_MAX_RPM = 6300.0;
        }

        public static final class BeamBreakConstants {
                public static final int BEAM_BREAK1 = 0;
                public static final int BEAM_BREAK2 = 1;
        }

        public static final class LimitSwitchConstants {
                public static final int LIMIT_SWITCH1_PORT = 3;
                public static final int LIMIT_SWITCH2_PORT = 2;
        }

        public static final class AdvancerConstants {

                public static final double ADVANCER_SPEED = -1.0;
                public static final double ADVANCER_ROLLER_SPEED = -0.5;
                public static final int ADVANCER_MOTOR_ID = 3;
        }

        public static final class IntakeConstants {
                public static final int INTAKE_MOTOR_ID = 3;
                /**Left */
                public static final int INTAKE_SLIDER1_ID = 5;
                /**Right */
                public static final int INTAKE_SLIDER2_ID = 6;
                public static final double INTAKE_SPEED = 1;
        }

        public static final class HoodConstants {
                public static final int HOOD_MOTOR_ID = 9;
                public static final boolean HOOD_INVERTED = false;
                public static final double HUB_X_METERS = 11.914;
                public static final double HUB_Y_METERS = 4.051;
                public static final double HOOD_MIN_DISTANCE_METERS = 1.0;
                public static final double HOOD_MAX_DISTANCE_METERS = 6.0;
                public static final double HOOD_MIN_ROTATIONS = 0.0;
                public static final double HOOD_MAX_ROTATIONS = 20.0;
                public static final double HOOD_HOME_ROTATIONS = 0.0;
                public static final double HOOD_KP = 0.08;
                public static final double HOOD_MAX_OUTPUT = 0.35;
                public static final double HOOD_POSITION_TOLERANCE_ROTATIONS = 0.25;
        }

        public static final class CimberConstants {
                public static final int CLIMBER_MOTOR_CANID = 8;
                public static final double Climber_Down_SetPoint = 380;
                public static final double Climber_Up_SetPoint = 20;
        }

        public static final class ShooterRPMConstants {
                public static final double DEPOT_SHOOT = 0.85;
                public static final double OUTSIDE_TRENCH = 0.85;
                public static final double INSIDE_TRENCH = 1;
        }

}
