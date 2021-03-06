import "package:p2/p2.dart" as p2;
import "renderer.dart";

main() {


        var stiffness = 100,
            damping = 5,
            restLength = 0.5;

        // Create demo application
        new WebGLRenderer((WebGLRenderer app){

            // Create the physics world
            var world = new p2.World(
                gravity : new p2.vec2(0.0,-10.0)
            );

            app.setWorld(world);

            // Set high friction so the wheels don't slip
            world.defaultContactMaterial.friction = 100;

            // Create ground
            var planeShape = new p2.Plane();
            var plane = new p2.Body();
            plane.addShape(planeShape);
            world.addBody(plane);

            // Create chassis
            var chassisBody = new p2.Body( mass : 1, position:new p2.vec2(0.0,1.0) ),
                chassisShape = new p2.Rectangle(1,0.5);
            chassisBody.addShape(chassisShape);
            world.addBody(chassisBody);

            // Create wheels
            var wheelBody1 = new p2.Body( mass : 1, position:new p2.vec2(chassisBody.position.x - 0.5,0.7) ),
                wheelBody2 = new p2.Body( mass : 1, position:new p2.vec2(chassisBody.position.x + 0.5,0.7) ),
                wheelShape = new p2.Circle(0.3);
            wheelBody1.addShape(wheelShape);
            wheelBody2.addShape(wheelShape);
            world.addBody(wheelBody1);
            world.addBody(wheelBody2);

            // Disable collisions between chassis and wheels
            var WHEELS =  1, // Define bits for each shape type
                CHASSIS = 2,
                GROUND =  4,
                OTHER =   8;

            wheelShape  .collisionGroup =   WHEELS; // Assign groups
            chassisShape.collisionGroup =   CHASSIS;
            planeShape  .collisionGroup =   GROUND;

            wheelShape  .collisionMask =    GROUND | OTHER;             // Wheels can only collide with ground
            chassisShape.collisionMask =    GROUND | OTHER;             // Chassis can only collide with ground
            planeShape  .collisionMask =    WHEELS | CHASSIS | OTHER;   // Ground can collide with wheels and chassis

            // Constrain wheels to chassis
            var c1 = new p2.PrismaticConstraint(chassisBody,wheelBody1,
                localAnchorA : new p2.vec2(-0.5,-0.3),
                localAnchorB : new p2.vec2(0.0,0.0),
                localAxisA : new p2.vec2(0.0,1.0),
                disableRotationalLock : true
            );
            var c2 = new p2.PrismaticConstraint(chassisBody,wheelBody2,
                localAnchorA : new p2.vec2( 0.5,-0.3),
                localAnchorB : new p2.vec2(0.0,0.0),
                localAxisA : new p2.vec2(0.0,1.0),
                disableRotationalLock : true
            );
            c1.setLimits(-0.4, 0.2);
            c2.setLimits(-0.4, 0.2);
            world.addConstraint(c1);
            world.addConstraint(c2);

            // Add springs for the suspension
            // Left spring
            world.addSpring(new p2.LinearSpring(chassisBody, wheelBody1, 
                restLength : restLength,
                stiffness : stiffness,
                damping : damping,
                localAnchorA : new p2.vec2(-0.5,0.0),
                localAnchorB : new p2.vec2(0.0,0.0)
            ));
            // Right spring
            world.addSpring(new p2.LinearSpring(chassisBody, wheelBody2, 
                restLength : restLength,
                stiffness : stiffness,
                damping : damping,
                localAnchorA : new p2.vec2(0.5,0.0),
                localAnchorB : new p2.vec2(0.0,0.0)
            ));

            app.newShapeCollisionGroup = OTHER;
            app.newShapeCollisionMask =  GROUND|WHEELS|CHASSIS|OTHER;

            // Apply current engine torque after each step
            var torque = 0;
            world.on("postStep",(evt){
                var max = 100;
                if(wheelBody1.angularVelocity*torque < max) wheelBody1.angularForce += torque;
                if(wheelBody2.angularVelocity*torque < max) wheelBody2.angularForce += torque;
            });

            // Change the current engine torque with the left/right keys
            app.on("keydown",(evt){
                num t = 5;
                switch(evt['keyCode']){
                    case 39: // right
                        torque = -t;
                        break;
                    case 37: // left
                        torque = t;
                        break;
                }
            });
            app.on("keyup",(evt){
                torque = 0;
            });

            world.on("addBody",(evt){
                evt['body'].setDensity(1);
            });
        });


}