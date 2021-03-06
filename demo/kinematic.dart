import "package:p2/p2.dart" as p2;
import "renderer.dart";
import "dart:math" as Math;

main() {


  // Create demo application
  new WebGLRenderer((WebGLRenderer app) {

    // Create a World
    var world = new p2.World(gravity: new p2.vec2(0.0, -10.0));

    app.setWorld(world);

    // Create ground
    var planeShape = new p2.Plane();
    var plane = new p2.Body(position: new p2.vec2(0.0, -2.0));
    plane.addShape(planeShape);
    world.addBody(plane);


    // Create kinematic, moving box
    p2.Body kinematicBody = new p2.Body(type: p2.Body.KINEMATIC, position: new p2.vec2(0.0, 0.5));
    var boxShape = new p2.Rectangle(2, 0.5);
    kinematicBody.addShape(boxShape);
    world.addBody(kinematicBody);


    // Create dynamic box
    var boxBody = new p2.Body(mass: 1, position: new p2.vec2(0.0, 2.0));
    boxBody.addShape(new p2.Rectangle(0.5, 0.5));
    world.addBody(boxBody);

    // Create dynamic circle connected to the kinematic body
    var circleBody = new p2.Body(mass: 1, position: new p2.vec2(0.0, -0.5), velocity: new p2.vec2(-1.0, 0.0));
    circleBody.addShape(new p2.Circle(0.25));
    world.addBody(circleBody);

    world.addConstraint(new p2.DistanceConstraint(kinematicBody, circleBody));

    world.on("postStep", (Map e) {
      // Kinematic bodies are controlled via velocity.
      kinematicBody.velocity.y = 2 * Math.sin(world.time * 2);
    });
  });


}
