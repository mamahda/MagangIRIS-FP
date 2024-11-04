import Ikm from "./Ikm.js";
import Fkm from "./Fkm.js";
import Encoder from "./Encoder.js";

class MotorLogic {
  static angle1 = 0;
  static angle2 = 120;
  static angle3 = 240;
  static DEG2RAD = 0.017453292519943295; // π / 180

  constructor(fkm) {
    this.fkm = fkm;
  }

  forwardKinematic() {
    const angle1Rad = MotorLogic.angle1 * MotorLogic.DEG2RAD;
    const angle2Rad = MotorLogic.angle2 * MotorLogic.DEG2RAD;
    const angle3Rad = MotorLogic.angle3 * MotorLogic.DEG2RAD;

    const vx =
      (2 / 3) *
      (this.fkm.motor1 * Math.cos(angle1Rad) +
        this.fkm.motor2 * Math.cos(angle2Rad) +
        this.fkm.motor3 * Math.cos(angle3Rad));

    const vy =
      (2 / 3) *
      (this.fkm.motor1 * Math.sin(angle1Rad) +
        this.fkm.motor2 * Math.sin(angle2Rad) +
        this.fkm.motor3 * Math.sin(angle3Rad));

    const vth = (this.fkm.motor1 + this.fkm.motor2 + this.fkm.motor3) / 3;

    return {
      vx: this.roundToZero(vx),
      vy: this.roundToZero(vy),
      vth: this.roundToZero(vth),
    };
  }

  inverseKinematic(target) {
    const angle1Rad = MotorLogic.angle1 * MotorLogic.DEG2RAD;
    const angle2Rad = MotorLogic.angle2 * MotorLogic.DEG2RAD;
    const angle3Rad = MotorLogic.angle3 * MotorLogic.DEG2RAD;

    let motor1 =
      target.vx * Math.cos(angle1Rad) +
      target.vy * Math.sin(angle1Rad) +
      target.vth;
    let motor2 =
      target.vx * Math.cos(angle2Rad) +
      target.vy * Math.sin(angle2Rad) +
      target.vth;
    let motor3 =
      target.vx * Math.cos(angle3Rad) +
      target.vy * Math.sin(angle3Rad) +
      target.vth;

    motor1 = this.roundToZero(motor1);
    motor2 = this.roundToZero(motor2);
    motor3 = this.roundToZero(motor3);

    return { motor1, motor2, motor3 };
  }

  roundToZero(value, tolerance = 0.00001) {
    return Math.abs(value) < tolerance
      ? 0
      : Math.round(value * 1000000) / 1000000;
  }
}

export default MotorLogic;
