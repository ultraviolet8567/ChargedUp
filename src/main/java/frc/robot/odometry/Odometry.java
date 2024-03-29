// // // feeds from gyro & vision
// // // calculates odometry data by taking weighted average from mentioned classes
// // // methods that output:
// // // robot translation (x, y, z)
// // // robot heading relative to the field

// // // things to do with one camera: find the camera transform3ds
// // // check odometry
// // // see if the get methods actually work

// package frc.robot.odometry;

// import java.util.ArrayList;
// import java.util.Optional;

// import org.littletonrobotics.junction.Logger;
// import org.photonvision.EstimatedRobotPose;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Odometry extends SubsystemBase {

//   GyroOdometry gyro;
//   VisionOdometry vision;

//   int count = 0;
//   ArrayList<Optional<EstimatedRobotPose>> visionDetections;

// //   // TODO: find the actual value
//   private final double framerate = 30;

//   private final double maxHeadingError = 0.02 * 2 * Math.PI; // number before * 2 * Math.PI is the maximum percent error allowed, tweak it

//   public Odometry(GyroOdometry gyro, VisionOdometry vision) {
//     this.gyro = gyro;
//     this.vision = vision;
//     this.visionDetections = new ArrayList<Optional<EstimatedRobotPose>>();
//   }

//   @Override
//   public void periodic() {
//     // updates gyro
//     gyro.updateGyroOdometry();

//     // CCW+
//     Logger.getInstance().recordOutput("Odometry/Heading", getHeading2d().getRadians());

//     try {
//       Logger.getInstance().recordOutput("Odometry/Values", vision.getTag());
//     } catch (NullPointerException e) {
//       // heyyy nothing's being detected!
//       Logger.getInstance().recordOutput("Odometry/Values", 23);
//     }
//   }

//   // i have no idea what this does
//   public double calculateVisionPercent() {
//     double errorScale = Math.abs(gyro.getRate()) / Units.degreesToRadians(8.0);
//     errorScale = MathUtil.clamp(errorScale, 0, 1); // errorscale is between 0 and 1
//     double shift = 1 - Math.pow(1 - 0.85, 1 / framerate); // i have no idea why this is happening but it's happening!
//     shift *= 1 - errorScale;

//     return shift;
//   }

//   // average of X-values
//   public double getX() {
//     try {
//       if (vision.checkCameras() >= 1) {
//         double error = Math.abs((vision.getHeading().getZ() - gyro.getHeading().getZ()) / gyro.getHeading().getZ());
//         if (error > maxHeadingError) {
//           return calculateAverage(vision.getX(), gyro.getX()); 
//         } else {
//           return gyro.getX(); 
//         }
//       } else {
//         return gyro.getX();
//       }
//     } catch (NullPointerException e) {
//       return gyro.getX();
//     }
//   }

//   // average of Y-values
//   public double getY() {
//     try {
//       if (vision.checkCameras() >= 1) {
//         double error = Math.abs((vision.getHeading().getZ() - gyro.getHeading().getZ()) / gyro.getHeading().getZ());
//         if (error > maxHeadingError) {
//           return calculateAverage(vision.getY(), gyro.getY()); 
//         } else {
//           return gyro.getY(); 
//         }
//       } else {
//         return gyro.getY();
//       }
//     } catch (NullPointerException e) {
//       return gyro.getY();
//     }
//   }

//   // depth
//   public double getZ() {
//     try {
//       if (vision.checkCameras() >= 1) {
//         return vision.getZ();
//       } else {
//         return 0.0;
//       }
//     } catch (NullPointerException e) {
//       return 0.0;
//     }
//   }

//   // calculates heading (angle) average
//   public Rotation3d getHeading() {
//     try {
//       if (vision.checkCameras() >= 1) {
//         double error = Math.abs((vision.getHeading().getZ() - gyro.getHeading().getZ()) / gyro.getHeading().getZ());
//         if (error > maxHeadingError) {
//           return calculateHeadingAverage(vision.getHeading(), gyro.getHeading()); 
//         } else {
//           return gyro.getHeading();
//         } 
//       } else {
//         return gyro.getHeading();
//       }
//     } catch (NullPointerException e) {
//       return gyro.getHeading();
//     }
//   }

//   public Rotation2d getHeading2d() {
//     return getHeading().toRotation2d();
//   }

//   // weighted average 
//   public double calculateAverage(double visionValue, double gyroValue) {
//     return calculateVisionPercent() * visionValue + (1 - calculateVisionPercent()) * gyroValue;
//   }

//   // weight average for heading
//   public Rotation3d calculateHeadingAverage(Rotation3d visionValue, Rotation3d gyroValue) {
//     double roll = calculateAverage(visionValue.getX(), gyroValue.getX());
//     double pitch = calculateAverage(visionValue.getY(), gyroValue.getY());
//     double yaw = calculateAverage(visionValue.getZ(), gyroValue.getZ());

//     return new Rotation3d(roll, pitch, yaw);
//   }

//   public Pose3d getPose3d() {
//     return new Pose3d(getX(), getY(), getZ(), getHeading());
//   }
// }