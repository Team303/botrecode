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
import edu.wpi.first.wpilibj.SPI;
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
