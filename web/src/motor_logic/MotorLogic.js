function _0x4a35() {
  const _0x2e0664 = [
    "11xYWLRY",
    "1568034pnrBFA",
    "angle2",
    "DEG2RAD",
    "159536rwjkaX",
    "fkm",
    "motor2",
    "4DVigiZ",
    "motor1",
    "inverseKinematic",
    "abs",
    "2236405OTNwYT",
    "vth",
    "sin",
    "192402gCiWhz",
    "round",
    "motor3",
    "angle3",
    "7853796jQgNFp",
    "2TXRkaZ",
    "cos",
    "angle1",
    "5392647AjMLZP",
    "forwardKinematic",
    "63TBHEAO",
    "16KpQfmZ",
    "roundToZero",
    "13377480vBzTUQ",
  ];
  _0x4a35 = function () {
    return _0x2e0664;
  };
  return _0x4a35();
}
function _0x58eb(_0x5b812a, _0x554cd1) {
  const _0x4a35c7 = _0x4a35();
  return (
    (_0x58eb = function (_0x58eb6f, _0x3ff97b) {
      _0x58eb6f = _0x58eb6f - 0x136;
      let _0x100a53 = _0x4a35c7[_0x58eb6f];
      return _0x100a53;
    }),
    _0x58eb(_0x5b812a, _0x554cd1)
  );
}
const _0x12ff7a = _0x58eb;
(function (_0x20ec4c, _0x3bd341) {
  const _0x2f61bb = _0x58eb,
    _0x44a764 = _0x20ec4c();
  while (!![]) {
    try {
      const _0x5a1278 =
        (parseInt(_0x2f61bb(0x13f)) / 0x1) *
          (-parseInt(_0x2f61bb(0x14e)) / 0x2) +
        parseInt(_0x2f61bb(0x13c)) / 0x3 +
        (-parseInt(_0x2f61bb(0x142)) / 0x4) *
          (parseInt(_0x2f61bb(0x146)) / 0x5) +
        (parseInt(_0x2f61bb(0x149)) / 0x6) *
          (parseInt(_0x2f61bb(0x137)) / 0x7) +
        (-parseInt(_0x2f61bb(0x138)) / 0x8) *
          (-parseInt(_0x2f61bb(0x151)) / 0x9) +
        -parseInt(_0x2f61bb(0x13a)) / 0xa +
        (-parseInt(_0x2f61bb(0x13b)) / 0xb) *
          (-parseInt(_0x2f61bb(0x14d)) / 0xc);
      if (_0x5a1278 === _0x3bd341) break;
      else _0x44a764["push"](_0x44a764["shift"]());
    } catch (_0x1079dc) {
      _0x44a764["push"](_0x44a764["shift"]());
    }
  }
})(_0x4a35, 0xafacd);
import _0xced4db from "./Ikm.js";
import _0x10c865 from "./Fkm.js";
import _0x4b7bad from "./Encoder.js";
class MotorLogic {
  static [_0x12ff7a(0x150)] = 0x0;
  static [_0x12ff7a(0x13d)] = 0x78;
  static [_0x12ff7a(0x14c)] = 0xf0;
  static [_0x12ff7a(0x13e)] = 0.017453292519943295;
  constructor(_0x45b8a0) {
    const _0x29ade9 = _0x12ff7a;
    this[_0x29ade9(0x140)] = _0x45b8a0;
  }
  [_0x12ff7a(0x136)]() {
    const _0x275d20 = _0x12ff7a,
      _0x387d6b = MotorLogic[_0x275d20(0x150)] * MotorLogic[_0x275d20(0x13e)],
      _0x2c90b8 = MotorLogic[_0x275d20(0x13d)] * MotorLogic[_0x275d20(0x13e)],
      _0x11ef5b = MotorLogic[_0x275d20(0x14c)] * MotorLogic[_0x275d20(0x13e)],
      _0x5f1f37 =
        (0x2 / 0x3) *
        (this[_0x275d20(0x140)][_0x275d20(0x143)] *
          Math[_0x275d20(0x14f)](_0x387d6b) +
          this[_0x275d20(0x140)][_0x275d20(0x141)] * Math["cos"](_0x2c90b8) +
          this[_0x275d20(0x140)][_0x275d20(0x14b)] * Math["cos"](_0x11ef5b)),
      _0x25829f =
        (0x2 / 0x3) *
        (this[_0x275d20(0x140)][_0x275d20(0x143)] * Math["sin"](_0x387d6b) +
          this[_0x275d20(0x140)][_0x275d20(0x141)] * Math["sin"](_0x2c90b8) +
          this[_0x275d20(0x140)][_0x275d20(0x14b)] *
            Math[_0x275d20(0x148)](_0x11ef5b)),
      _0x5badbd =
        (this["fkm"][_0x275d20(0x143)] +
          this[_0x275d20(0x140)]["motor2"] +
          this["fkm"][_0x275d20(0x14b)]) /
        0x3,
      _0x50bccc = this[_0x275d20(0x139)](_0x5f1f37),
      _0x3c9568 = this[_0x275d20(0x139)](_0x25829f),
      _0x5381b6 = this["roundToZero"](_0x5badbd);
    return { vx: _0x50bccc, vy: _0x3c9568, vth: _0x5381b6 };
  }
  [_0x12ff7a(0x144)](_0x23a7b0) {
    const _0x46fc9b = _0x12ff7a,
      _0x1429b1 = MotorLogic[_0x46fc9b(0x150)] * MotorLogic[_0x46fc9b(0x13e)],
      _0x5b962c = MotorLogic[_0x46fc9b(0x13d)] * MotorLogic["DEG2RAD"],
      _0x285576 = MotorLogic[_0x46fc9b(0x14c)] * MotorLogic["DEG2RAD"];
    let _0x2d6950 =
        _0x23a7b0["vx"] * Math[_0x46fc9b(0x14f)](_0x1429b1) +
        _0x23a7b0["vy"] * Math[_0x46fc9b(0x148)](_0x1429b1) +
        _0x23a7b0[_0x46fc9b(0x147)],
      _0x5a4e13 =
        _0x23a7b0["vx"] * Math[_0x46fc9b(0x14f)](_0x5b962c) +
        _0x23a7b0["vy"] * Math[_0x46fc9b(0x148)](_0x5b962c) +
        _0x23a7b0["vth"],
      _0xf515a7 =
        _0x23a7b0["vx"] * Math["cos"](_0x285576) +
        _0x23a7b0["vy"] * Math["sin"](_0x285576) +
        _0x23a7b0["vth"];
    return (
      (_0x2d6950 = this["roundToZero"](_0x2d6950)),
      (_0x5a4e13 = this[_0x46fc9b(0x139)](_0x5a4e13)),
      (_0xf515a7 = this[_0x46fc9b(0x139)](_0xf515a7)),
      { motor1: _0x2d6950, motor2: _0x5a4e13, motor3: _0xf515a7 }
    );
  }
  [_0x12ff7a(0x139)](_0x3b2d8a, _0xb40ed6 = 0.00001) {
    const _0x1beb86 = _0x12ff7a;
    if (Math[_0x1beb86(0x145)](_0x3b2d8a) < _0xb40ed6) return 0x0;
    return Math[_0x1beb86(0x14a)](_0x3b2d8a * 0xf4240) / 0xf4240;
  }
}
export default MotorLogic;
