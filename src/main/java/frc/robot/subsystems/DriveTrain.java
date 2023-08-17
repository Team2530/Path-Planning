package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

    private final WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.kLEFT_FRONT_CAN_ID);
    private final WPI_TalonFX leftRear = new WPI_TalonFX(DriveConstants.kLEFT_REAR_CAN_ID);
    private final WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.kRIGHT_FRONT_CAN_ID);
    private final WPI_TalonFX rightRear = new WPI_TalonFX(DriveConstants.kRIGHT_REAR_CAN_ID);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftRear);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightRear);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
    private final DifferentialDrivetrainSim driveSimulation = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2),
            DriveConstants.kGearRatio, DriveConstants.kDriveTrainMomentOfInertia,
            Units.lbsToKilograms(112), Units.inchesToMeters(DriveConstants.kWheelRadiusInches),
            DriveConstants.kTrackWidth, null);

    private final static Gyro navX = new AHRS(SPI.Port.kMXP);

    private final DifferentialDriveOdometry odometry;

    private Field2d field = new Field2d();

    public DriveTrain() {
        // Configure rear motors to follow front ones
        // leftRear.follow(leftFront);
        // rightRear.follow(rightFront);

        // Invert right side of drivetrain
        leftFront.setInverted(false);
        leftRear.setInverted(false);
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        // leftFront.setInverted(false);
        // leftRear.setInverted(true);
        // rightFront.setInverted(false);
        // rightRear.setInverted(true);

        // Reset Encoder positions
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);

        // Set motor mode to braked
        rightFront.setNeutralMode(NeutralMode.Brake);
        leftFront.setNeutralMode(NeutralMode.Brake);
        rightRear.setNeutralMode(NeutralMode.Brake);
        leftRear.setNeutralMode(NeutralMode.Brake);

        // Try calibrating NavX, and report if failed
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                navX.reset();
                navX.calibrate();
                System.out.println("NavX Recalibrating...");
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                System.out.println("Sleep Failed, NavX may not be calibrated fully/correctly");
                // e.printStackTrace();
            }
        }).start();

        odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftDistanceMeters(),
                getRightDistanceMeters());

        odometry.resetPosition(navX.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d());
        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        SmartDashboard.putData(drive);

    }

    @Override
    public void periodic() {
        odometry.update(navX.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Left Encoder", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Encoder", getRightDistanceMeters());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("Left Speed", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("Right Speed", getWheelSpeeds().rightMetersPerSecond);
    }

    /**
     * Stops drive motors
     */
    public void stop() {
        leftMotors.set(0);
        rightMotors.set(0);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    public void resetEncoders() {
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
        leftRear.setSelectedSensorPosition(0);
        rightRear.setSelectedSensorPosition(0);
    }

    public double getLeftDistanceMeters() {
        return -leftFront.getSelectedSensorPosition() * DriveConstants.kLinearDistanceConversionFactor / 2048d;
    }

    public double getRightDistanceMeters() {
        return -rightFront.getSelectedSensorPosition() * DriveConstants.kLinearDistanceConversionFactor / 2048d;
    }

    /* Meters / minute */
    public double getLeftVelocity() {
        return -leftFront.getSelectedSensorVelocity() * DriveConstants.kLinearDistanceConversionFactor / 60d / 2048d;
    }

    /* Meters / minute */
    public double getRightVelocity() {
        return -rightFront.getSelectedSensorVelocity() * DriveConstants.kLinearDistanceConversionFactor / 60d / 2048d;
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -navX.getRate();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(navX.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), pose);
    }

    public static void zeroHeading() {
        navX.reset();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(-leftVolts);
        rightMotors.setVoltage(-rightVolts);
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    public double getAverageLeftMeters() {
        return -((leftFront.getSelectedSensorPosition() * DriveConstants.kLinearDistanceConversionFactor) +
                (leftRear.getSelectedSensorPosition() * DriveConstants.kLinearDistanceConversionFactor)) / 2.0;
    }

    public double getAverageRightMeters() {
        return ((rightFront.getSelectedSensorPosition() * DriveConstants.kLinearDistanceConversionFactor) +
                (rightRear.getSelectedSensorPosition() * DriveConstants.kLinearDistanceConversionFactor)) / 2.0;
    }

    public void setDriveMode(NeutralMode mode) {
        rightFront.setNeutralMode(mode);
        leftFront.setNeutralMode(mode);
        rightRear.setNeutralMode(mode);
        leftRear.setNeutralMode(mode);
    }

    /**
     * Only for simulation!
     * 
     * @param meters
     */
    private void setLeftDistanceMeters(double meters) {
        leftFront.setSelectedSensorPosition(meters / DriveConstants.kLinearDistanceConversionFactor * 2048d);
        leftRear.setSelectedSensorPosition(meters / DriveConstants.kLinearDistanceConversionFactor * 2048d);
    }

    /**
     * Only for simulation!
     * 
     * @param meters
     */
    private void setRightDistanceMeters(double meters) {
        rightFront.setSelectedSensorPosition(meters / DriveConstants.kLinearDistanceConversionFactor * 2048d);
        rightRear.setSelectedSensorPosition(meters / DriveConstants.kLinearDistanceConversionFactor * 2048d);

    }

    @Override
    public void simulationPeriodic() {
        driveSimulation.setInputs(
                leftMotors.get() * RobotController.getInputVoltage(),
                rightMotors.get() * RobotController.getInputVoltage());
        driveSimulation.update(0.02);

        setLeftDistanceMeters(driveSimulation.getLeftPositionMeters());
        setRightDistanceMeters(driveSimulation.getRightPositionMeters());

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(driveSimulation.getHeading().getDegrees());

    }
}
