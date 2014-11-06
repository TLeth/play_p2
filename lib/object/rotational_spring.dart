part of p2;

class RotationalSpring extends Spring {

  /// Rest angle of the spring.
  num restAngle;

  RotationalSpring(Body bodyA, Body bodyB, {num restAngle, num stiffness: 100, num damping: 1})
      : super(bodyA, bodyB, stiffness: stiffness, damping: damping) {
    this.restAngle = restAngle != null ? restAngle : bodyB.angle - bodyA.angle;
  }

  /// Apply the spring force to the connected bodies.

  applyForce() {
    num k = this.stiffness;
    num l = this.restAngle;
    num d = this.damping;
    Body bodyA = this.bodyA;
    Body bodyB = this.bodyB;
    num x = bodyB.angle - bodyA.angle;
    num u = bodyB.angularVelocity - bodyA.angularVelocity;

    num torque = -k * (x - l) - d * u * 0;

    bodyA.angularForce -= torque;
    bodyB.angularForce += torque;
  }
}
