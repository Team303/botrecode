package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.modules.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule leftFrontModule;
    private final SwerveModule rightFrontModule;
    private final SwerveModule leftBackModule;
    private final SwerveModule rightBackModule;

    private final SwerveDriveKinematics kinematics;
    private final AHRS navx;
    private final SwerveDriveOdometry odometry;

    /* Shuffleboard Debugging */
    private final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
    private GenericEntry[] moduleAngleEntries = new GenericEntry[4];
    private GenericEntry[] moduleVelocityEntries = new GenericEntry[4];
    private GenericEntry robotHeadingEntry;
    private GenericEntry robotPoseXEntry;
    private GenericEntry robotPoseYEntry;

    public SwerveSubsystem() {
        navx = new AHRS(SPI.Port.kMXP);
        leftFrontModule = new SwerveModule(DriveConstants.LEFT_FRONT_DRIVE_ID,
                DriveConstants.LEFT_FRONT_STEER_ID,
                DriveConstants.LEFT_FRONT_STEER_CANCODER_ID);
        rightFrontModule = new SwerveModule(
                DriveConstants.RIGHT_FRONT_DRIVE_ID,
                DriveConstants.RIGHT_FRONT_STEER_ID,
                DriveConstants.RIGHT_FRONT_STEER_CANCODER_ID);
        leftBackModule = new SwerveModule(
                DriveConstants.LEFT_BACK_DRIVE_ID,
                DriveConstants.LEFT_BACK_STEER_ID,
                DriveConstants.LEFT_BACK_STEER_CANCODER_ID);
        rightBackModule = new SwerveModule(
                DriveConstants.RIGHT_BACK_DRIVE_ID,
                DriveConstants.RIGHT_BACK_STEER_ID,
                DriveConstants.RIGHT_BACK_STEER_CANCODER_ID);

        kinematics = new SwerveDriveKinematics(
                new Translation2d(DriveConstants.WHEELBASE / 2, DriveConstants.TRACKWIDTH / 2),
                new Translation2d(DriveConstants.WHEELBASE / 2, -DriveConstants.TRACKWIDTH / 2),
                new Translation2d(-DriveConstants.WHEELBASE / 2, DriveConstants.WHEELBASE / 2),
                new Translation2d(-DriveConstants.WHEELBASE / 2, -DriveConstants.WHEELBASE / 2));

        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), new SwerveModulePosition[] {
                leftFrontModule.getPosition(), rightFrontModule.getPosition(), leftBackModule.getPosition(),
                rightBackModule.getPosition()
        });

        initializeShuffleboard();
    }

    @Override
    public void periodic() {
        odometry.update(
                getRotation2d(),
                new SwerveModulePosition[] {
                        leftFrontModule.getPosition(),
                        rightFrontModule.getPosition(),
                        leftBackModule.getPosition(),
                        rightBackModule.getPosition()
                });

        updateShuffleboard();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
                leftFrontModule.getPosition(), rightFrontModule.getPosition(), leftBackModule.getPosition(),
                rightBackModule.getPosition()
        }, pose);
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        leftFrontModule.setDesiredState(desiredStates[0]);
        rightFrontModule.setDesiredState(desiredStates[1]);
        leftBackModule.setDesiredState(desiredStates[2]);
        rightBackModule.setDesiredState(desiredStates[3]);
    }

    public void setXPattern() {
        SwerveModuleState[] xStates = new SwerveModuleState[4];
        xStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        xStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        xStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        xStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        setModuleStates(xStates);
    }

    private void initializeShuffleboard() {
        robotHeadingEntry = swerveTab.add("Robot Heading", 0).getEntry();
        robotPoseXEntry = swerveTab.add("Robot Pose X", 0).getEntry();
        robotPoseYEntry = swerveTab.add("Robot Pose Y", 0).getEntry();

        String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };
        for (int i = 0; i < 4; i++) {
            moduleAngleEntries[i] = swerveTab.add(moduleNames[i] + " Angle", 0).getEntry();
            moduleVelocityEntries[i] = swerveTab.add(moduleNames[i] + " Velocity", 0).getEntry();
        }
    }

    private void updateShuffleboard() {
        Pose2d pose = getPose();
        robotHeadingEntry.setDouble(getHeading());
        robotPoseXEntry.setDouble(pose.getX());
        robotPoseYEntry.setDouble(pose.getY());

        SwerveModule[] modules = { leftFrontModule, rightFrontModule, leftBackModule, rightBackModule };
        for (int i = 0; i < 4; i++) {
            moduleAngleEntries[i].setDouble(modules[i].getSteerAngle().getDegrees());
            moduleVelocityEntries[i].setDouble(modules[i].getDriveVelocity());
        }
    }

    private Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    public void zeroHeading() {
        navx.zeroYaw();
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public double getPitch() {
        return navx.getPitch();
    }

    public double getRoll() {
        return navx.getRoll();
    }

}
