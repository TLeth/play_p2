part of p2;

class Convex extends Shape {

  /// Vertices defined in the local frame.
  List vertices;

  /// Axes defined in the local frame.
  List axes;

  /// The center of mass of the Convex
  List centerOfMass;

  /// Triangulated version of this convex. The structure is Array of 3-Arrays, and each subarray contains 3 integers, referencing the vertices.
  List triangles;

  Convex([List vertices, List axes]) : super(Shape.CONVEX) {

    this.vertices = [];

    this.axes = [];

    // Copy the verts
    for (int i = 0; i < vertices.length; i++) {
      List v = vec2.create();
      vec2.copy(v, vertices[i]);
      this.vertices.add(v);
    }

    if (axes != null) {
      // Copy the axes
      for (var i = 0; i < axes.length; i++) {
        List axis = vec2.create();
        vec2.copy(axis, axes[i]);
        this.axes.add(axis);
      }
    } else {
      // Construct axes from the vertex data
      for (var i = 0; i < vertices.length; i++) {
        // Get the world edge
        List worldPoint0 = vertices[i];
        List worldPoint1 = vertices[(i + 1) % vertices.length];

        List normal = vec2.create();
        vec2.sub(normal, worldPoint1, worldPoint0);

        // Get normal - just rotate 90 degrees since vertices are given in CCW
        vec2.rotate90cw(normal, normal);
        vec2.normalize(normal, normal);

        this.axes.add(normal);
      }
    }


    this.centerOfMass = vec2.fromValues(0, 0);

    this.triangles = [];

    if (this.vertices.length) {
      this.updateTriangles();
      this.updateCenterOfMass();
    }

    /**
     * The bounding radius of the convex
     * @property boundingRadius
     * @type {Number}
     */
    this.boundingRadius = 0;

    //Shape.call(this, Shape.CONVEX);

    this.updateBoundingRadius();
    this.updateArea();
    if (this.area < 0) {
      throw new Exception("Convex vertices must be given in conter-clockwise winding.");
    }
  }

  List tmpVec1 = vec2.create();
  List tmpVec2 = vec2.create();

  /**
   * Project a Convex onto a world-oriented axis
   * @method projectOntoAxis
   * @static
   * @param  {Array} offset
   * @param  {Array} localAxis
   * @param  {Array} result
   */

  projectOntoLocalAxis(List localAxis, List result) {
    var max = null,
    min = null,
    v,
    value,
    localAxis = tmpVec1;

    // Get projected position of all vertices
    for (int i = 0; i < this.vertices.length; i++) {
      v = this.vertices[i];
      value = vec2.dot(v, localAxis);
      if (max == null || value > max) {
        max = value;
      }
      if (min == null || value < min) {
        min = value;
      }
    }

    if (min > max) {
      var t = min;
      min = max;
      max = t;
    }

    vec2.set(result, min, max);
  }

  projectOntoWorldAxis(List localAxis, shapeOffset, shapeAngle, result) {
    var worldAxis = tmpVec2;

    this.projectOntoLocalAxis(localAxis, result);

    // Project the position of the body onto the axis - need to add this to the result
    if (shapeAngle != 0) {
      vec2.rotate(worldAxis, localAxis, shapeAngle);
    } else {
      worldAxis = localAxis;
    }
    var offset = vec2.dot(shapeOffset, worldAxis);

    vec2.set(result, result[0] + offset, result[1] + offset);
  }


  /**
   * Update the .triangles property
   * @method updateTriangles
   */

  updateTriangles() {

    this.triangles.length = 0;

    // Rewrite on polyk notation, array of numbers
    List polykVerts = [];
    for (var i = 0; i < this.vertices.length; i++) {
      var v = this.vertices[i];
      polykVerts.addAll([v[0], v[1]]);
    }

    // Triangulate
    List triangles = Polyk.Triangulate(polykVerts);

    // Loop over all triangles, add their inertia contributions to I
    for (int i = 0; i < triangles.length; i += 3) {
      var id1 = triangles[i],
      id2 = triangles[i + 1],
      id3 = triangles[i + 2];

      // Add to triangles
      this.triangles.add([id1, id2, id3]);
    }
  }

  var updateCenterOfMass_centroid = vec2.create(),
  updateCenterOfMass_centroid_times_mass = vec2.create(),
  updateCenterOfMass_a = vec2.create(),
  updateCenterOfMass_b = vec2.create(),
  updateCenterOfMass_c = vec2.create(),
  updateCenterOfMass_ac = vec2.create(),
  updateCenterOfMass_ca = vec2.create(),
  updateCenterOfMass_cb = vec2.create(),
  updateCenterOfMass_n = vec2.create();

  /**
   * Update the .centerOfMass property.
   * @method updateCenterOfMass
   */

  updateCenterOfMass() {
    var triangles = this.triangles,
    verts = this.vertices,
    cm = this.centerOfMass,
    centroid = updateCenterOfMass_centroid,
    n = updateCenterOfMass_n,
    a = updateCenterOfMass_a,
    b = updateCenterOfMass_b,
    c = updateCenterOfMass_c,
    ac = updateCenterOfMass_ac,
    ca = updateCenterOfMass_ca,
    cb = updateCenterOfMass_cb,
    centroid_times_mass = updateCenterOfMass_centroid_times_mass;

    vec2.set(cm, 0, 0);
    num totalArea = 0;

    for (var i = 0; i != triangles.length; i++) {
      var t = triangles[i],
      a = verts[t[0]],
      b = verts[t[1]],
      c = verts[t[2]];

      vec2.centroid(centroid, a, b, c);

      // Get mass for the triangle (density=1 in this case)
      // http://math.stackexchange.com/questions/80198/area-of-triangle-via-vectors
      var m = Convex.triangleArea(a, b, c);
      totalArea += m;

      // Add to center of mass
      vec2.scale(centroid_times_mass, centroid, m);
      vec2.add(cm, cm, centroid_times_mass);
    }

    vec2.scale(cm, cm, 1 / totalArea);
  }

  /**
   * Compute the mass moment of inertia of the Convex.
   * @method computeMomentOfInertia
   * @param  {Number} mass
   * @return {Number}
   * @see http://www.gamedev.net/topic/342822-moment-of-inertia-of-a-polygon-2d/
   */

  computeMomentOfInertia(num mass) {
    var denom = 0.0,
    numer = 0.0,
    N = this.vertices.length;
    for (var j = N - 1, i = 0; i < N; j = i, i ++) {
      var p0 = this.vertices[j];
      var p1 = this.vertices[i];
      var a = (vec2.crossLength(p0, p1)).abs();
      var b = vec2.dot(p1, p1) + vec2.dot(p1, p0) + vec2.dot(p0, p0);
      denom += a * b;
      numer += a;
    }
    return (mass / 6.0) * (denom / numer);
  }

  /**
   * Updates the .boundingRadius property
   * @method updateBoundingRadius
   */

  updateBoundingRadius() {
    var verts = this.vertices,
    r2 = 0;

    for (var i = 0; i != verts.length; i++) {
      var l2 = vec2.squaredLength(verts[i]);
      if (l2 > r2) {
        r2 = l2;
      }
    }

    this.boundingRadius = sqrt(r2);
  }

  /**
   * Get the area of the triangle spanned by the three points a, b, c. The area is positive if the points are given in counter-clockwise order, otherwise negative.
   * @static
   * @method triangleArea
   * @param {Array} a
   * @param {Array} b
   * @param {Array} c
   * @return {Number}
   */

  static triangleArea(List a, List b, List c) {
    return (((b[0] - a[0]) * (c[1] - a[1])) - ((c[0] - a[0]) * (b[1] - a[1]))) * 0.5;
  }

  /**
   * Update the .area
   * @method updateArea
   */

  updateArea() {
    this.updateTriangles();
    this.area = 0;

    List triangles = this.triangles,
    verts = this.vertices;
    for (var i = 0; i != triangles.length; i++) {
      var t = triangles[i],
      a = verts[t[0]],
      b = verts[t[1]],
      c = verts[t[2]];

      // Get mass for the triangle (density=1 in this case)
      num m = Convex.triangleArea(a, b, c);
      this.area += m;
    }
  }

  /**
   * @method computeAABB
   * @param  {AABB}   out
   * @param  {Array}  position
   * @param  {Number} angle
   */

  computeAABB(AABB out, [List position, num angle]) {
    out.setFromPoints(this.vertices, position, angle, 0);
  }
}
