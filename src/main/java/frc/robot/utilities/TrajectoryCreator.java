package frc.robot.utilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class TrajectoryCreator {
    private File file;
    private FileWriter writer;
    private Timer timer;
    private double previousTime;
    private double previousVelocity;
    private boolean firstDatapoint = true;

    public TrajectoryCreator(String filename) {
        try {
            // Create a file
            file = new File(filename);
            file.createNewFile();

            // Create a writer to write to the file
            writer = new FileWriter(filename);

            // Writing the header of the file
            createFileHeader();

            timer = new Timer();
            timer.reset();
            timer.start();

            previousTime = 0;
            previousVelocity = 0;
        } catch (IOException e) {
            DriverStation.reportError("Failed to open file: " + file.getAbsolutePath(), e.getStackTrace());
        }
    }

    private double calculateAcceleration(double velocity, double time) {
        return (velocity - previousVelocity) / (time - previousTime);
    }

    private double calculateCurvature(double velocity, double angularVelocity) {
        if (velocity == 0) {return 0;};
        return angularVelocity / velocity;
    }

    public void addDatapoint(ChassisSpeeds speeds, Pose2d pose) {
        try {
            if ((Math.abs(speeds.vxMetersPerSecond) >= Constants.Autonomous.VELOCITY_THRESHOLD_START) && firstDatapoint) {
                timer.stop();
                timer.reset();
                timer.start();
                previousTime = -0.1;
            } else if ((Math.abs(speeds.vxMetersPerSecond) < Constants.Autonomous.VELOCITY_THRESHOLD_START) && firstDatapoint) {
                return;
            } else if (Math.abs(speeds.vxMetersPerSecond) < Constants.Autonomous.VELOCITY_THRESHOLD_END) {
                timer.stop();
                return;
            }
            
            double cTime = timer.get();

            if (!firstDatapoint) {
                writer.write(",\n");
            }

            writer.write("{\n");
            writer.write("\"acceleration\": " + calculateAcceleration(speeds.vxMetersPerSecond, cTime) + ",\n");
            writer.write("\"curvature\": " + calculateCurvature(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond) + ",\n");
            writer.write("\"pose\": {\n");
            writer.write("\"rotation\": {\n");
            writer.write("\"radians\": " + pose.getRotation().getRadians() + "\n");
            writer.write("},\n");
            writer.write("\"translation\": {\n");
            writer.write("\"x\": " + pose.getX() * Constants.Autonomous.POSITION_CORRECTION + ",\n");
            writer.write("\"y\": " + pose.getY() * Constants.Autonomous.POSITION_CORRECTION + "\n");
            writer.write("}\n");
            writer.write("},\n");
            writer.write("\"time\": " + cTime + ",\n");
            writer.write("\"velocity\": " + speeds.vxMetersPerSecond + "\n");
            writer.write("}");

            previousTime = cTime;
            previousVelocity = speeds.vxMetersPerSecond;
            firstDatapoint = false;

        } catch (IOException e) {
            DriverStation.reportError("Failed to write datapoint: " + file, e.getStackTrace());
        }
    }

    private void createFileHeader() {
        try {
            writer.write("[\n");
        } catch (IOException e) {
            DriverStation.reportError("Failed to write file: " + file, e.getStackTrace());
        }
    }

    private void createFileFooter() {
        try {
            writer.write("\n]\n");
        } catch (IOException e) {
            DriverStation.reportError("Failed to write file: " + file, e.getStackTrace());
        }
    }

    public void close() {
        try {
            // Writing the footer of the file
            createFileFooter();

            // Writing the content to the file
            writer.close();
        } catch (IOException e) {
            DriverStation.reportError("Unable to write file: " + file.getAbsolutePath(), e.getStackTrace());
        }
    }
}