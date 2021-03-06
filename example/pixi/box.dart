import "dart:html";
import "package:play_pixi/pixi.dart" as PIXI;
import "package:p2/p2.dart" as p2;


var renderer, stage, container, graphics, zoom,
world, boxShape, boxBody, planeBody, planeShape;

//init();
//animate();

init() {

  // Init p2.js
  world = new p2.World();

  // Add a box
  boxShape = new p2.Rectangle(2, 1);
  boxBody = new p2.Body(
      mass:1,
      position:new p2.vec2(0.0, 2.0),
      angularVelocity:1
  );
  boxBody.addShape(boxShape);
  world.addBody(boxBody);

  // Add a plane
  planeShape = new p2.Plane();
  planeBody = new p2.Body(
      position:new p2.vec2(0.0, -1.0)
  );
  planeBody.addShape(planeShape);
  world.addBody(planeBody);

  // Pixi.js zoom level
  zoom = 100;

  // Initialize the stage
  renderer = PIXI.autoDetectRenderer(600, 400);
  stage = new PIXI.Stage(0xFFFFFF);

// We use a container inside the stage for all our content
// This enables us to zoom and translate the content
  container = new PIXI.DisplayObjectContainer();
  stage.addChild(container);

// Add the canvas to the DOM
  document.body.children.add(renderer.view);

// Add transform to the container
  container.position.x = renderer.width / 2; // center at origin
  container.position.y = renderer.height / 2;
  container.scale.x = zoom; // zoom in
  container.scale.y = -zoom; // Note: we flip the y axis to make "up" the physics "up"

// Draw the box.
  graphics = new PIXI.Graphics();
  graphics.beginFill(0xff0000);
  graphics.drawRect(-boxShape.width / 2, -boxShape.height / 2, boxShape.width, boxShape.height);

// Add the box to our container
  container.addChild(graphics);
}

// Animation loop

animate(num t) {
  //t = t || 0;
  PIXI.requestAnimFrame(animate);

  // Move physics bodies forward in time
  world.step(1 / 60);

  // Transfer positions of the physics objects to Pixi.js
  graphics.position.x = boxBody.position[0];
  graphics.position.y = boxBody.position[1];
  graphics.rotation = boxBody.angle;

  // Render scene
  renderer.render(stage);
}