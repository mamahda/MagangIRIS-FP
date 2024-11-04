import MotorLogic from "./MotorLogic.js";
import Fkm from "./Fkm.js";
import Encoder from "./Encoder.js";
import Pose from "./Pose.js";

let pose = new Pose(0.0, 100, 0.0);
let fkm = new Fkm(0.0, 86.60254, -86.60254);

// let fkm2 = new Fkm(100.000000, -50.000000, -50.000000)
let logic1 = new MotorLogic(fkm);
let enc = new Encoder(pose);

// let masuk = logic1.forwardKinematic(fkm);
let output = logic1.inverseKinematic(masuk);
// // let encod = enc.inverseKinematicEncoder(masuk);

let output_encoder = enc.inverseKinematicEncoder(pose);
