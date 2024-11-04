import Ikm from "./Ikm.js";
import Odometry from "./Odometry.js";
import Pose from "./Pose.js";

class Encoder {
  // Static properties
  static angle1 = 315; // Angle1 value in degrees
  static angle2 = 45; // Angle2 value in degrees
  static DEG2RAD = 0.017453292519943295; // Conversion factor from degrees to radians

  constructor(pose) {
    this.pose = pose; // Set initial pose from the constructor
  }

  // Method to compute inverse kinematic encoder values
  inverseKinematicEncoder() {
    const angle1Rad = Encoder.angle1 * Encoder.DEG2RAD;
    const angle2Rad = Encoder.angle2 * Encoder.DEG2RAD;

    // Calculations for encoder values
    let enc_left =
      this.pose.x * Math.cos(angle1Rad) + this.pose.y * Math.sin(angle1Rad);
    let enc_right =
      this.pose.x * Math.cos(angle2Rad) + this.pose.y * Math.sin(angle2Rad);
    const th = this.pose.th; // Pose angle theta

    // Rounding encoder values to avoid floating-point errors
    enc_left = this.roundToZero(enc_left);
    enc_right = this.roundToZero(enc_right);

    return { enc_left, enc_right, th };
  }

  // Method to round values close to zero
  roundToZero(value, tolerance = 0.00001) {
    if (Math.abs(value) < tolerance) return 0;
    return Math.round(value * 1000000) / 1000000; // Round to six decimal places
  }
}

export default Encoder;
