const _0x20dd40 = _0x362b;
function _0x27dc() {
  const _0x3fe180 = [
    "5oprHHA",
    "515576SVAjZS",
    "8396269FBotgm",
    "610dKXMHZ",
    "cos",
    "roundToZero",
    "4642077XaoirM",
    "1011126TenzBM",
    "round",
    "DEG2RAD",
    "sin",
    "320342TosClX",
    "63jPEmUj",
    "6651942HFWcCy",
    "pose",
    "abs",
    "angle2",
    "4xGzmCz",
    "619328HmACLn",
  ];
  _0x27dc = function () {
    return _0x3fe180;
  };
  return _0x27dc();
}
(function (_0x46b40b, _0x544d70) {
  const _0x16858a = _0x362b,
    _0x58d948 = _0x46b40b();
  while (!![]) {
    try {
      const _0x301181 =
        -parseInt(_0x16858a(0x1d0)) / 0x1 +
        parseInt(_0x16858a(0x1ca)) / 0x2 +
        (-parseInt(_0x16858a(0x1cf)) / 0x3) *
          (-parseInt(_0x16858a(0x1c7)) / 0x4) +
        (parseInt(_0x16858a(0x1c9)) / 0x5) *
          (-parseInt(_0x16858a(0x1d6)) / 0x6) +
        -parseInt(_0x16858a(0x1cb)) / 0x7 +
        (-parseInt(_0x16858a(0x1c8)) / 0x8) *
          (-parseInt(_0x16858a(0x1d5)) / 0x9) +
        (-parseInt(_0x16858a(0x1cc)) / 0xa) *
          (-parseInt(_0x16858a(0x1d4)) / 0xb);
      if (_0x301181 === _0x544d70) break;
      else _0x58d948["push"](_0x58d948["shift"]());
    } catch (_0x582bc9) {
      _0x58d948["push"](_0x58d948["shift"]());
    }
  }
})(_0x27dc, 0xc459b);
import _0x1492f4 from "./Ikm.js";
import _0x27ac19 from "./Odometry.js";
function _0x362b(_0x3cdb61, _0x4c5156) {
  const _0x27dc83 = _0x27dc();
  return (
    (_0x362b = function (_0x362b03, _0x267bea) {
      _0x362b03 = _0x362b03 - 0x1c7;
      let _0x1a8a1e = _0x27dc83[_0x362b03];
      return _0x1a8a1e;
    }),
    _0x362b(_0x3cdb61, _0x4c5156)
  );
}
import _0x431df2 from "./Pose.js";
class Encoder {
  static ["angle1"] = 0x13b;
  static [_0x20dd40(0x1d9)] = 0x2d;
  static [_0x20dd40(0x1d2)] = 0.017453292519943295;
  constructor(_0xb9930b) {
    const _0x1f1642 = _0x20dd40;
    this[_0x1f1642(0x1d7)] = _0xb9930b;
  }
  ["inverseKinematicEncoder"]() {
    const _0x379d1f = _0x20dd40,
      _0x537a0a = Encoder["angle1"] * Encoder[_0x379d1f(0x1d2)],
      _0x5558e7 = Encoder[_0x379d1f(0x1d9)] * Encoder["DEG2RAD"];
    let _0x4e4477 =
        this[_0x379d1f(0x1d7)]["x"] * Math[_0x379d1f(0x1cd)](_0x537a0a) +
        this["pose"]["y"] * Math[_0x379d1f(0x1d3)](_0x537a0a),
      _0x60af88 =
        this[_0x379d1f(0x1d7)]["x"] * Math[_0x379d1f(0x1cd)](_0x5558e7) +
        this["pose"]["y"] * Math[_0x379d1f(0x1d3)](_0x5558e7),
      _0x236606 = this["pose"]["th"];
    return (
      (_0x4e4477 = this[_0x379d1f(0x1ce)](_0x4e4477)),
      (_0x60af88 = this["roundToZero"](_0x60af88)),
      { enc_left: _0x4e4477, enc_right: _0x60af88, th: _0x236606 }
    );
  }
  ["roundToZero"](_0x1f78e4, _0x188c53 = 0.00001) {
    const _0x52cbff = _0x20dd40;
    if (Math[_0x52cbff(0x1d8)](_0x1f78e4) < _0x188c53) return 0x0;
    return Math[_0x52cbff(0x1d1)](_0x1f78e4 * 0xf4240) / 0xf4240;
  }
}
export default Encoder;
