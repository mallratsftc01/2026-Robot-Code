package frc.robot.subsystems.drivetrainIOLayers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.CoordinateConverter;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;
import frc.robot.Constants.SwerveConstants.Mod4;
import frc.robot.model.MetricName;
import frc.robot.service.MetricService;

import static frc.robot.Constants.SwerveConstants.*;

import java.io.FileWriter;
import java.io.IOException;

import java.nio.file.Paths;
import java.nio.file.Files;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class DrivetrainIO extends SubsystemBase {

  private ShuffleboardTab drivetrain_tab = Shuffleboard.getTab("DriveTrain");

  private GenericEntry driveRot_entry = drivetrain_tab.add("drive rot", 0)
      .getEntry();
  private GenericEntry driveX_entry = drivetrain_tab.add("drive x", 0)
      .getEntry();
  private GenericEntry driveY_entry = drivetrain_tab.add("drive y", 0)
      .getEntry();
  private GenericEntry gyro_entry = drivetrain_tab.add("gyro", 0)
      .getEntry();

  private static final boolean shouldLogToFile = false;

  private SwerveModule backLeft_0 = new SwerveModule(Mod3.constants);
  private SwerveModule backRight_1 = new SwerveModule(Mod4.constants);
  private SwerveModule frontRight_2 = new SwerveModule(Mod1.constants);
  private SwerveModule frontLeft_3 = new SwerveModule(Mod2.constants);

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

  private double xSpeed_cur;
  private double ySpeed_cur;
  private double rot_cur;

  private double targetHeading;
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_backLeftLocation,
      m_backRightLocation,
      m_frontRightLocation, m_frontLeftLocation);

  private final SwerveDrivePoseEstimator poseEstimator;
  private long lastVisionUpdateTime = 0;

  // @SuppressWarnings("unused")
  // private final SwerveSetpointGenerator setpointGenerator;
  @SuppressWarnings("unused")
  private SwerveSetpoint previousSetpoint;
  private RobotConfig config;

  private final FileWriter logFileWriter;

  public double getAngle() {
    return gyro.getAngle();
  }

  public DrivetrainIO() {

    try {
      // make sure the folders exist
      Files.createDirectories(Paths.get("/home/lvuser"));
      logFileWriter = new FileWriter("/home/lvuser/drivelog.txt", false);
      logFileWriter
          .write(
              "time,gyro,m0distance,m0degrees,m1distance,m1degrees,m2distance,m2degrees,m3distance,m3degrees,estX,estY,estDegrees, \n");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    ResetGyro();
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    poseEstimator.resetPosition(CoordinateConverter.convertToAllianceRotation(new Rotation2d(-90)),
        getModulePositions(),
        CoordinateConverter.convertToAllianceCoordinates(new Pose2d(7.558, 4.010, new Rotation2d(-90))));

    configureAutoBuilder();

    // setpointGenerator = new SwerveSetpointGenerator(
    // config,
    // Units.rotationsToRadians(1.0) // The max rotation velocity of a swerve module
    // in radians per second. This
    // // should probably be stored in your Constants file
    // );
    // previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(),
    // DriveFeedforwards.zeros(config.numModules));
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        backLeft_0.getState(),
        backRight_1.getState(),
        frontRight_2.getState(),
        frontLeft_3.getState(),
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        backLeft_0.getPosition(),
        backRight_1.getPosition(),
        frontRight_2.getPosition(),
        frontLeft_3.getPosition(),
    };
  }

  double Rotate_Rot = 0.0;

  public void Rotate_Rot(double r) {
    Rotate_Rot = r;
  }

  public void driveRobotRelative(ChassisSpeeds c) {
    drive((c.vxMetersPerSecond / MaxMetersPersecond),
        (c.vyMetersPerSecond / MaxMetersPersecond), (c.omegaRadiansPerSecond / kModuleMaxAngularVelocity), false);
  }

  public void driveRobotRelativeMetersPerSecond(ChassisSpeeds speeds) {
    var discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxMetersPersecond);
    setModuleStates(swerveModuleStates);
  }

  int timeDelay = 0;

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    driveRot_entry.setDouble(rot + Rotate_Rot);
    driveX_entry.setDouble(xSpeed);
    driveY_entry.setDouble(ySpeed);
    MetricService.publish(MetricName.REQUESTED_SPEED_X, xSpeed);
    MetricService.publish(MetricName.REQUESTED_SPEED_Y, ySpeed);
    MetricService.publish(MetricName.REQUESTED_ROTATION, rot);
    xSpeed_cur = xSpeed;
    ySpeed_cur = ySpeed;
    rot_cur = rot + Rotate_Rot;
    gyro_entry.setDouble(gyro.getAngle());

    var sourceChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot + Rotate_Rot, gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    var chassisSpeeds = ChassisSpeeds.discretize(sourceChassisSpeeds, 0.02);

    SmartDashboard.putNumber("[Drivetrain]chassis rot", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("[Drivetrain]chassis xSpeed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("[Drivetrain]chassis ySpeed", chassisSpeeds.vyMetersPerSecond);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxMetersPersecond);

    SmartDashboard.putNumber("[Drivetrain]gyro", gyro.getRotation2d().getRadians());
    // SmartDashboard.putString("[Drivetrain]module 0",
    // swerveModuleStates[0].toString());
    // SmartDashboard.putString("[Drivetrain]module 1",
    // swerveModuleStates[1].toString());
    // SmartDashboard.putString("[Drivetrain]module 2",
    // swerveModuleStates[2].toString());
    // SmartDashboard.putString("[Drivetrain]module 3",
    // swerveModuleStates[3].toString());
    // if (xSpeed + ySpeed + rot == 0 && !DriverStation.isAutonomous()) {
    // timeDelay++;
    // if (timeDelay >= 50) {
    // setModuleStates(new SwerveModuleState[] {
    // new SwerveModuleState(0, new Rotation2d(-45)),
    // new SwerveModuleState(0, new Rotation2d(45)),
    // new SwerveModuleState(0, new Rotation2d(-45)),
    // new SwerveModuleState(0, new Rotation2d(45)),
    // });
    // }
    // } else {
    // timeDelay = 0;
    setModuleStates(swerveModuleStates);
    // }
  }

  public void driveRobotRelativeWithFF(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    var discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxMetersPersecond);

    backLeft_0.SetDesiredStateWithFF(swerveModuleStates[0],
        feedforwards.linearForces()[0].in(edu.wpi.first.units.Units.Newtons));
    backRight_1.SetDesiredStateWithFF(swerveModuleStates[1],
        feedforwards.linearForces()[1].in(edu.wpi.first.units.Units.Newtons));
    frontRight_2.SetDesiredStateWithFF(swerveModuleStates[2],
        feedforwards.linearForces()[2].in(edu.wpi.first.units.Units.Newtons));
    frontLeft_3.SetDesiredStateWithFF(swerveModuleStates[3],
        feedforwards.linearForces()[3].in(edu.wpi.first.units.Units.Newtons));
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    backLeft_0.SetDesiredState(swerveModuleStates[0]);
    backRight_1.SetDesiredState(swerveModuleStates[1]);
    frontRight_2.SetDesiredState(swerveModuleStates[2]);
    frontLeft_3.SetDesiredState(swerveModuleStates[3]);
  }

  public void ResetGyro() {
    gyro.reset();
    // we need to start at this offset otherwise our axis are reversed
    // note: docs say gyro should increase counterclockwise. ours decreases. may
    // need to look into this.
    // gyro.setAngleAdjustment(CoordinateConverter.convertToAllianceRotation(Rotation2d.fromDegrees(-90)).getDegrees());
    gyro.setAngleAdjustment(DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(-90).getDegrees()
        : Rotation2d.fromDegrees(-90).getDegrees());
    SmartDashboard.putString("[Drivetrain]Gyro has been reset", java.time.LocalTime.now().toString());
    System.out.println("Gyro has been reset");
  }

  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds(xSpeed_cur, ySpeed_cur, rot_cur);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    var c = m_kinematics.toChassisSpeeds(getModuleStates());
    SmartDashboard.putString("[Drivetrain]Robot relative speeds", c.toString());
    return c;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d aPose2d) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), aPose2d);
  }

  public void configureAutoBuilder() {
    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto hasa starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelativeWithFF(speeds, feedforwards), // Method that will drive the robot
                                                                                  // given ROBOT
        // RELATIVE
        // ChassisSpeeds. Also optionally outputs
        // individual
        // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
            // holonomic
            // drive trains
            new PIDConstants(2, 0, 0.1), // Translation PID constants
            new PIDConstants(0.2, 0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public PathConstraints getChassisConstrains() {
    return getChassisConstraints();
  }

  public PathConstraints getChassisConstraints() {
    return new PathConstraints(
        3.000,
        3.000,
        Units.degreesToRadians(540.000),
        Units.degreesToRadians(720.000));
  }

  /**
   * See
   * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    lastVisionUpdateTime = System.currentTimeMillis();
  }

  public long visionUpdateDelayMillis() {
    return System.currentTimeMillis() - lastVisionUpdateTime;
  }

  private String incomingLog = "";

  public void log(String s) {
    if (shouldLogToFile) {
      incomingLog += s;
    }
  }

  @Override
  public void periodic() {
    var gyroRot = gyro.getRotation2d();
    var modulePos = getModulePositions();
    poseEstimator.update(gyroRot, modulePos);
    if (shouldLogToFile) {
      try {
        var pose = poseEstimator.getEstimatedPosition();
        logFileWriter
            .write(System.currentTimeMillis() + "," + gyroRot.getDegrees() + "," + modulePos[0].distanceMeters + ","
                + modulePos[0].angle.getDegrees() + ","
                + modulePos[1].distanceMeters + "," + modulePos[1].angle.getDegrees() + ","
                + modulePos[2].distanceMeters + "," + modulePos[2].angle.getDegrees() + ","
                + modulePos[3].distanceMeters
                + "," + modulePos[3].angle.getDegrees() + "," + pose.getX() + "," + pose.getY() + ","
                + pose.getRotation().getDegrees() + ","
                + incomingLog + "\n");

      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

    SmartDashboard.putNumber("Vision Update delay", visionUpdateDelayMillis());
  }

}
