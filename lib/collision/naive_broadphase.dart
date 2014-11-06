part of p2;

/// Naive broadphase implementation. Does N^2 tests.
class NaiveBroadphase extends Broadphase {
  NaiveBroadphase() : super(Broadphase.NAIVE);

  /// Get the colliding pairs
  List<Body> getCollisionPairs(World world) {
    List<Body> bodies = world.bodies;
    List<Body> result = this.result;

    result.clear();

    {
      int i = 0;
      int Ncolliding = bodies.length;
      for ( ; i != Ncolliding; i++) {
        Body bi = bodies[i];

        for (int j = 0; j < i; j++) {
          Body bj = bodies[j];

          if (Broadphase.canCollide(bi, bj) && this.boundingVolumeCheck(bi, bj)) {
            result.add(bi);
            result.add(bj);
          }
        }
      }
    }

    return result;
  }
}
