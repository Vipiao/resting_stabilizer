

let debug = 0;

// Vector2 class for 2D math operations
class Vector2 {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }
    
    static add(a, b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }
    
    static subtract(a, b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }
    
    static multiply(v, scalar) {
        return new Vector2(v.x * scalar, v.y * scalar);
    }
    
    static dot(a, b) {
        return a.x * b.x + a.y * b.y;
    }
    
    static cross(a, b) {
        return a.x * b.y - a.y * b.x;
    }
    
    static perpendicular(v) {
        return new Vector2(-v.y, v.x);
    }
    
    static normalize(v) {
        const length = Math.sqrt(v.x * v.x + v.y * v.y);
        if (length === 0) return new Vector2(0, 0);
        return new Vector2(v.x / length, v.y / length);
    }
    
    static length(v) {
        return Math.sqrt(v.x * v.x + v.y * v.y);
    }
    
    copy() {
        return new Vector2(this.x, this.y);
    }
}

// Rectangle physics body
class Rectangle {
    constructor(x, y, width, height, mass = 1.0, isStatic = false) {
        this.position = new Vector2(x, y);
        this.velocity = new Vector2(0, 0);
        this.deltaVelocity = new Vector2(0, 0);
        
        this.angle = 0;
        this.angularVelocity = 0;
        this.deltaAngularVelocity = 0;
        
        this.width = width;
        this.height = height;
        
        this.isStatic = isStatic;
        this.mass = isStatic ? Infinity : mass;
        this.invMass = isStatic ? 0 : 1.0 / mass;
        
        // Moment of inertia for rectangle: (1/12) * mass * (width² + height²)
        this.inertia = isStatic ? Infinity : (mass * (width * width + height * height)) / 12.0;
        this.invInertia = isStatic ? 0 : 1.0 / this.inertia;

        // Friction coefficients
        //this.staticFriction = isStatic ? 0.9 : 0.6;
        //this.dynamicFriction = isStatic ? 0.7 : 0.4;
        this.staticFriction = isStatic ? 0.9 : 0.6;
        this.dynamicFriction = isStatic ? 0.7 : 0.4;
        
        this.id = Rectangle.nextId++;
        this.color = this.generateRandomColor();
    }
    
    static nextId = 0;
    
    generateRandomColor() {
        if (this.isStatic) return '#444444';
        const hue = random() * 360;
        return `hsl(${hue}, 70%, 60%)`;
    }
    
    getVertices() {
        const cos = Math.cos(this.angle);
        const sin = Math.sin(this.angle);
        const halfW = this.width / 2;
        const halfH = this.height / 2;
        
        const vertices = [
            new Vector2(-halfW, -halfH),
            new Vector2(halfW, -halfH),
            new Vector2(halfW, halfH),
            new Vector2(-halfW, halfH)
        ];
        
        // Rotate and translate vertices
        return vertices.map(v => {
            const rotatedX = v.x * cos - v.y * sin;
            const rotatedY = v.x * sin + v.y * cos;
            return new Vector2(rotatedX + this.position.x, rotatedY + this.position.y);
        });
    }
    
    applyImpulse(impulse, contactPoint) {
        if (this.isStatic) return;
        
        this.velocity = Vector2.add(this.velocity, Vector2.multiply(impulse, this.invMass));
        
        const r = Vector2.subtract(contactPoint, this.position);
        const torque = Vector2.cross(r, impulse);
        this.angularVelocity += torque * this.invInertia;
    }
    
    getVelocityAtPoint(point) {
        const r = Vector2.subtract(point, this.position);
        const tangentialVel = new Vector2(-r.y * this.angularVelocity, r.x * this.angularVelocity);
        return Vector2.add(this.velocity, tangentialVel);
    }
}

// Collision contact information
class Contact {
    constructor(bodyA, bodyB, contactPoints, normal, penetrations) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.contactPoints = contactPoints; // Array of contact points, one per contact point
        this.normal = normal;
        this.tangent = Vector2.normalize(Vector2.perpendicular(normal));
        this.penetrations = penetrations; // Array of penetrations, one per contact point
        this.collisionMasses = []; // Array of collision masses, one per contact point
        this.tangentialCollisionMasses = []; // Array of tangential collision masses
        this.collisionMassesCalculated = []; // Array of booleans, one per contact point
        this.tangentialCollisionMassesCalculated = []; // Array of booleans for tangential
        this.normalForceMagnitudes = []; // Array to store normal force magnitudes from resting phase

        // Initialize arrays
        for (let i = 0; i < contactPoints.length; i++) {
            this.collisionMasses.push(0);
            this.tangentialCollisionMasses.push(0);
            this.collisionMassesCalculated.push(false);
            this.tangentialCollisionMassesCalculated.push(false);
            this.normalForceMagnitudes.push(0);
        }
    }
}

// Persistent contact cache entry
class ContactCache {
    constructor(bodyA, bodyB) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.restingVelocityA = new Vector2(0, 0);
        this.restingVelocityB = new Vector2(0, 0);
        this.restingAngularVelocityA = 0;
        this.restingAngularVelocityB = 0;
        this.lastUpdateTime = 0;
        this.key = this.generateKey(bodyA, bodyB);
    }
    
    generateKey(bodyA, bodyB) {
        const minId = Math.min(bodyA.id, bodyB.id);
        const maxId = Math.max(bodyA.id, bodyB.id);
        return `${minId}_${maxId}`;
    }
}

// Deterministic random number generator using LCG
class DeterministicRandom {
        constructor(seed = 12345) {
            this.seed = seed;
        }
        
        // LCG parameters (same as used by glibc)
        next() {
            this.seed = (this.seed * 1103515245 + 12345) & 0x7fffffff;
            return this.seed / 0x7fffffff; // Return 0-1 range
        }
        
        // Reset to initial seed
        reset(seed = 12345) {
            this.seed = seed;
        }
    }
    
    // Global deterministic random instance
    const detRandom = new DeterministicRandom();
    
    // Replace random() calls with this
    function random() {
        return detRandom.next();
    }

// Main physics engine
class PhysicsEngine {
    constructor(worldWidth, worldHeight) {
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
        this.hz = 60;
        this.bodies = [];
        this.contacts = [];
        this.contactCache = new Map();
        this.gravity = new Vector2(0, 1 / this.hz); // pixels/tick²
        this.isPaused = false;
        this.currentTime = 0;
        this.cacheTimeout = this.hz * 0.5; // 0.5 seconds worth of ticks

        // Mouse interaction state
        this.mousePosition = new Vector2(0, 0);
        this.grabbedBody = null;
        this.grabOffset = new Vector2(0, 0);
        this.isMouseDown = false;
        
        this.initializeWorld();
    }
    
    initializeWorld() {
        // Create static ground
        const ground = new Rectangle(this.worldWidth / 2, this.worldHeight - 25, this.worldWidth, 50, 1, true);
        this.bodies.push(ground);
        
        // Create some random boxes
        for (let i = 0; i < 1; i++) {
            //this.addRandomBox();

            const box = new Rectangle(200, 200, 50, 30, 1, false);
            box.angle = 0.1;
            this.bodies.push(box);
        }
    }
    
    addRandomBox() {
        const width = 30 + random() * 40;
        const height = 30 + random() * 40;
        const x = 200 + random() * 100;
        const y = 50 + random() * 200;
        const mass = 0.5 + random() * 2.0;
        
        const box = new Rectangle(x, y, width, height, mass, false);
        // Add some initial random velocity
        box.velocity.x = (random() - 0.5) * 1;
        box.velocity.y = (random() - 0.5) * 5;
        box.angularVelocity = (random() - 0.5) * 0.5;
        
        this.bodies.push(box);
    }
    
    update() {
        if (this.isPaused) return;
        
        this.currentTime += 1;
        
        // 1. Collision detection and contact generation
        this.detectCollisions();
        
        // 2. Reset delta velocities
        this.resetDeltaVelocities();
        
        // 3. Apply continuous forces (gravity)
        this.applyForces();
        
        // 4. Move objects according to velocities
        this.integrateMotion();
        for (const contact of this.contacts) {
            // Calculate collision mass for each contact point
            for (let i = 0; i < contact.contactPoints.length; i++) {
                this.calculateCollisionMass(contact, i);
            }
        }
        
        // 5. Position correction (separate overlapping objects)
        this.separateObjects();
        
        // 6. Apply resting forces
        this.applyRestingForces();
        
        // 7. Measure error and correct
        this.measureAndCorrectError();
        
        // 8. Apply normal collision resolution
        this.applyNormalCollision();

        // 9. Handle mouse interactions
        this.updateMouseInteraction();

        // 10. Apply resting decay.
        this.applyRestingDecay();
        
        // Clean up old cache entries
        this.cleanupContactCache();
    }
    
    detectCollisions() {
        this.contacts = [];
        
        for (let i = 0; i < this.bodies.length; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const bodyA = this.bodies[i];
                const bodyB = this.bodies[j];
                
                const collision = this.checkSATCollision(bodyA, bodyB);
                if (collision) {
                    this.contacts.push(collision);
                    
                    // Update or create contact cache
                    const cacheKey = new ContactCache(bodyA, bodyB).key;
                    if (!this.contactCache.has(cacheKey)) {
                        this.contactCache.set(cacheKey, new ContactCache(bodyA, bodyB));
                    }
                    this.contactCache.get(cacheKey).lastUpdateTime = this.currentTime;
                }
            }
        }
    }
    
    checkSATCollision(rectA, rectB) {
        debug++;
        const verticesA = rectA.getVertices();
        const verticesB = rectB.getVertices();
        
        let minOverlap = Infinity;
        let minOverlapA = Infinity;
        let minOverlapB = Infinity;
        let separationAxis = null;
        let minAxisA = null;
        let minAxisB = null;
        let projA_minAxisA;
        let projB_minAxisB;
        
        // Test axes from both rectangles
        let axisA = this.getRectangleAxes(verticesA);
        let axisB = this.getRectangleAxes(verticesB);
        
        for (const axis of axisA) {
            const projA = this.projectVertices(verticesA, axis);
            const projB = this.projectVertices(verticesB, axis);
            
            const overlap = Math.min(projA.max, projB.max) - Math.max(projA.min, projB.min);
            
            if (overlap < 0) {
                return null; // No collision
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                separationAxis = axis.copy();
            }
            
            if (overlap < minOverlapA) {
                minOverlapA = overlap;
                minAxisA = axis.copy();
                projA_minAxisA = projA;
            }
        }
        for (const axis of axisB) {
            const projA = this.projectVertices(verticesA, axis);
            const projB = this.projectVertices(verticesB, axis);
            
            const overlap = Math.min(projA.max, projB.max) - Math.max(projA.min, projB.min);
            
            if (overlap < 0) {
                return null; // No collision
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                separationAxis = axis.copy();
            }
            
            if (overlap < minOverlapB) {
                minOverlapB = overlap;
                minAxisB = axis.copy();
                projB_minAxisB = projB;
            }
        }
        
        // Find contact points by checking vertices of each rectangle against the other
        const contactPointsA = this.findContactPoints(verticesA, rectB);
        const contactPointsB = this.findContactPoints(verticesB, rectA);
                
        // Ensure normal points from A to B
        const centerA = this.getCenter(verticesA);
        const centerB = this.getCenter(verticesB);
        const centerDir = Vector2.subtract(centerB, centerA);
        if (Vector2.dot(separationAxis, centerDir) < 0) {
            separationAxis = Vector2.multiply(separationAxis, -1);
        }
        
        const contactPoints = [...contactPointsA, ...contactPointsB];
        
        // If no contact points found, use fallback
        if (contactPoints.length === 0) {
            contactPoints.push(Vector2.multiply(Vector2.add(rectA.position, rectB.position), 0.5));
        }

        // Calculate penetration per contact point
        const penetration = [];
        for (const contactPoint of contactPoints) {

            let pointProjectionA = Vector2.dot(contactPoint, minAxisA);
            let minDistToA = Math.abs(pointProjectionA - projA_minAxisA.min);
            minDistToA = Math.min(minDistToA, Math.abs(pointProjectionA - projA_minAxisA.max));
            
            let pointProjectionB = Vector2.dot(contactPoint, minAxisB);
            let minDistToB = Math.abs(pointProjectionB - projB_minAxisB.min);
            minDistToB = Math.min(minDistToB, Math.abs(pointProjectionB - projB_minAxisB.max));

            penetration.push(Math.max(minDistToA, minDistToB));
        }
        
        return new Contact(rectA, rectB, contactPoints, separationAxis, penetration);
    }
    
    getRectangleAxes(vertices) {
        const axes = [];
        for (let i = 0; i < vertices.length; i++) {
            const edge = Vector2.subtract(vertices[(i + 1) % vertices.length], vertices[i]);
            const normal = Vector2.normalize(Vector2.perpendicular(edge));
            axes.push(normal);
        }
        return axes;
    }
    
    projectVertices(vertices, axis) {
        let min = Vector2.dot(vertices[0], axis);
        let max = min;
        
        for (let i = 1; i < vertices.length; i++) {
            const projection = Vector2.dot(vertices[i], axis);
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
        
        return { min, max };
    }
    
    getCenter(vertices) {
        let sum = new Vector2(0, 0);
        for (const vertex of vertices) {
            sum = Vector2.add(sum, vertex);
        }
        return Vector2.multiply(sum, 1.0 / vertices.length);
    }
    
    findContactPoints(vertices, rect) {
        const contactPoints = [];

        // Check if any vertex is inside the rectangle
        const rectVertices = rect.getVertices();
        for (const vertex of vertices) {
            if (this.isPointInRectangle(vertex, rectVertices)) {
                contactPoints.push(vertex);
            }
        }
        
        return contactPoints;
    }
    
    isPointInRectangle(point, rectVertices) {
        // Simple point-in-polygon test using cross products
        for (let i = 0; i < rectVertices.length; i++) {
            const v1 = rectVertices[i];
            const v2 = rectVertices[(i + 1) % rectVertices.length];
            const edge = Vector2.subtract(v2, v1);
            const toPoint = Vector2.subtract(point, v1);
            
            if (Vector2.cross(edge, toPoint) < 0) {
                return false;
            }
        }
        return true;
    }
    
    resetDeltaVelocities() {
        for (const body of this.bodies) {
            body.deltaVelocity = new Vector2(0, 0);
            body.deltaAngularVelocity = 0;
        }
    }
    
    applyForces() {
        const gravityDelta = this.gravity;
        
        for (const body of this.bodies) {
            if (!body.isStatic) {
                body.velocity = Vector2.add(body.velocity, gravityDelta);
                body.deltaVelocity = Vector2.add(body.deltaVelocity, gravityDelta);
            }
        }
    }
    
    integrateMotion() {
        for (const body of this.bodies) {
            if (!body.isStatic) {
                body.position = Vector2.add(body.position, body.velocity);
                body.angle += body.angularVelocity;
            }
        }
    }
    
    separateObjects() {
        for (const contact of this.contacts) {
            // Process each contact point
            for (let i = 0; i < contact.contactPoints.length; i++) {
                const contactPoint = contact.contactPoints[i];
                const collisionMass = contact.collisionMasses[i];
                const penetration = contact.penetrations[i];
                let adjustedPenetration = Math.max(0, penetration) - 2;
                if(adjustedPenetration < 0) adjustedPenetration *= 0.0;
                
                // Calculate displacement using collision mass (like impulse but for position)
                const displacement = Vector2.multiply(contact.normal, adjustedPenetration * collisionMass * 0.3);
                
                // Apply displacement to position
                if (!contact.bodyA.isStatic) {
                    contact.bodyA.position = Vector2.subtract(contact.bodyA.position, Vector2.multiply(displacement, contact.bodyA.invMass));
                    const rA = Vector2.subtract(contactPoint, contact.bodyA.position);
                    contact.bodyA.angle -= Vector2.cross(rA, displacement) * contact.bodyA.invInertia;
                }
                
                if (!contact.bodyB.isStatic) {
                    contact.bodyB.position = Vector2.add(contact.bodyB.position, Vector2.multiply(displacement, contact.bodyB.invMass));
                    const rB = Vector2.subtract(contactPoint, contact.bodyB.position);
                    contact.bodyB.angle += Vector2.cross(rB, displacement) * contact.bodyB.invInertia;
                }
            }
        }
    }
    
    applyRestingForces() {
        // Phase 1: Use (delta + resting) velocities, apply to both real and delta
        for (const contact of this.contacts) {
            const cache = this.getContactCache(contact.bodyA, contact.bodyB);
            
            // Process each contact point
            for (let i = 0; i < contact.contactPoints.length; i++) {
                // Calculate combined velocities (delta + resting)
                //const combinedVelA = Vector2.subtract(contact.bodyA.deltaVelocity, cache.restingVelocityA);
                //const combinedVelB = Vector2.subtract(contact.bodyB.deltaVelocity, cache.restingVelocityB);
                //const combinedAngVelA = contact.bodyA.deltaAngularVelocity - cache.restingAngularVelocityA;
                //const combinedAngVelB = contact.bodyB.deltaAngularVelocity - cache.restingAngularVelocityB;
                
                //const combinedVelA = Vector2.multiply(cache.restingVelocityA, -1);
                //const combinedVelB = Vector2.multiply(cache.restingVelocityB, -1);
                //const combinedAngVelA = - cache.restingAngularVelocityA;
                //const combinedAngVelB = - cache.restingAngularVelocityB;

                const combinedVelA = Vector2.subtract(contact.bodyA.velocity, cache.restingVelocityA);
                const combinedVelB = Vector2.subtract(contact.bodyB.velocity, cache.restingVelocityB);
                const combinedAngVelA = contact.bodyA.angularVelocity - cache.restingAngularVelocityA;
                const combinedAngVelB = contact.bodyB.angularVelocity - cache.restingAngularVelocityB;
                
                // Normal force
                const impulse = this.calculateImpulse(contact, i, combinedVelA, combinedVelB, combinedAngVelA, combinedAngVelB, 
                    contact.collisionMasses[i], contact.normal, false);

                // Store normal force magnitude for later phases
                contact.normalForceMagnitudes[i] = Vector2.length(impulse);

                // Friction force
                const frictionImpulse = this.calculateImpulse(contact, i, combinedVelA, combinedVelB, combinedAngVelA, combinedAngVelB,
                    contact.tangentialCollisionMasses[i], contact.tangent, true);
                const combinedStaticFriction = Math.min(contact.bodyA.staticFriction, contact.bodyB.staticFriction);
                const combinedDynamicFriction = Math.min(contact.bodyA.dynamicFriction, contact.bodyB.dynamicFriction);
                const maxStaticFriction = contact.normalForceMagnitudes[i] * combinedStaticFriction;
                
                // Combine impulses and apply once
                let totalImpulse = impulse;
                if (Vector2.length(frictionImpulse) <= maxStaticFriction) {
                    totalImpulse = Vector2.add(impulse, frictionImpulse);
               } else {
                   // Apply dynamic friction
                   const dynamicFriction = contact.normalForceMagnitudes[i] * combinedDynamicFriction;
                   const frictionDirection = Vector2.normalize(frictionImpulse);
                   const dynamicFrictionImpulse = Vector2.multiply(frictionDirection, dynamicFriction);
                   totalImpulse = Vector2.add(impulse, dynamicFrictionImpulse);
                }

                // Apply combined impulse to both real and delta velocities
                // contact, contactPointIndex, impulse, applyToReal, applyToDelta, applyToResting
                this.applyImpulseToVelocities(contact, i, totalImpulse, true, false, false);
            }
        }
    }
    
    measureAndCorrectError() {
        // Phase 2: Use delta velocities, apply to delta and resting
        for (const contact of this.contacts) {
            const cache = this.getContactCache(contact.bodyA, contact.bodyB);
            
            // Process each contact point
            for (let i = 0; i < contact.contactPoints.length; i++) {
                // Normal force
                const impulse = this.calculateImpulse(contact, i,
                    contact.bodyA.velocity, contact.bodyB.velocity,
                    contact.bodyA.angularVelocity, contact.bodyB.angularVelocity,
                    contact.collisionMasses[i], contact.normal, true);
                
                // Friction force
                const frictionImpulse = this.calculateImpulse(contact, i,
                    contact.bodyA.velocity, contact.bodyB.velocity,
                    contact.bodyA.angularVelocity, contact.bodyB.deltaAngularVelocity,
                    contact.tangentialCollisionMasses[i], contact.tangent, true);
                const combinedStaticFriction = Math.min(contact.bodyA.staticFriction, contact.bodyB.staticFriction);
                const combinedDynamicFriction = Math.min(contact.bodyA.dynamicFriction, contact.bodyB.dynamicFriction);
                const maxStaticFriction = (contact.normalForceMagnitudes[i] + Vector2.length(impulse)) * combinedStaticFriction;
                
                // Combine impulses and apply once
                let totalImpulse = impulse;
                if (Vector2.length(frictionImpulse) <= maxStaticFriction) {
                    totalImpulse = Vector2.add(impulse, frictionImpulse);
               } else {
                   // Apply dynamic friction
                   const dynamicFriction = (contact.normalForceMagnitudes[i] + Vector2.length(impulse)) * combinedDynamicFriction;
                   const frictionDirection = Vector2.normalize(frictionImpulse);
                   const dynamicFrictionImpulse = Vector2.multiply(frictionDirection, dynamicFriction);
                   totalImpulse = Vector2.add(impulse, dynamicFrictionImpulse);
                }
                
                // Apply combined impulse to delta velocities and resting
                // contact, contactPointIndex, impulse, applyToReal, applyToDelta, applyToResting
                this.applyImpulseToVelocities(contact, i, totalImpulse, false, false, true);
            }
        }
    }
    
    applyNormalCollision() {
        // Phase 3: Use real velocities, apply to real velocities
        for (const contact of this.contacts) {
            // Process each contact point
            for (let i = 0; i < contact.contactPoints.length; i++) {
                // Normal collision
                const impulse = this.calculateImpulse(contact, i,
                    contact.bodyA.velocity, contact.bodyB.velocity,
                    contact.bodyA.angularVelocity, contact.bodyB.angularVelocity,
                    contact.collisionMasses[i], contact.normal, false);

                // Friction collision
                const frictionImpulse = this.calculateImpulse(contact, i,
                    contact.bodyA.velocity, contact.bodyB.velocity,
                    contact.bodyA.angularVelocity, contact.bodyB.angularVelocity,
                    contact.tangentialCollisionMasses[i], contact.tangent, true);
                
                const combinedStaticFriction = Math.min(contact.bodyA.staticFriction, contact.bodyB.staticFriction);
                const combinedDynamicFriction = Math.min(contact.bodyA.dynamicFriction, contact.bodyB.dynamicFriction);
                const maxStaticFriction = (contact.normalForceMagnitudes[i] + Vector2.length(impulse)) * combinedStaticFriction;
                
                // Combine impulses and apply once
                let totalImpulse = impulse;
                if (Vector2.length(frictionImpulse) <= maxStaticFriction) {
                    totalImpulse = Vector2.add(impulse, frictionImpulse);
                } else {
                    // Apply dynamic friction
                    const dynamicFriction = (contact.normalForceMagnitudes[i] + Vector2.length(impulse)) * combinedDynamicFriction;
                    const frictionDirection = Vector2.normalize(frictionImpulse);
                    const dynamicFrictionImpulse = Vector2.multiply(frictionDirection, dynamicFriction);
                    totalImpulse = Vector2.add(impulse, dynamicFrictionImpulse);
                }

                // Apply combined impulse to real velocities
                // contact, contactPointIndex, impulse, applyToReal, applyToDelta, applyToResting
                this.applyImpulseToVelocities(contact, i, totalImpulse, true, false, false);
            }
        }
    }
    
    calculateCollisionMass(contact, contactPointIndex) {
        if (contact.collisionMassesCalculated[contactPointIndex]) return;
        
        const contactPoint = contact.contactPoints[contactPointIndex];
        
        const rA = Vector2.subtract(contactPoint, contact.bodyA.position);
        const rB = Vector2.subtract(contactPoint, contact.bodyB.position);
        
        // Normal collision mass
        const rAcrossN = Vector2.cross(rA, contact.normal);
        const rBcrossN = Vector2.cross(rB, contact.normal);
        
        const rotTermA = rAcrossN * rAcrossN * contact.bodyA.invInertia;
        const rotTermB = rBcrossN * rBcrossN * contact.bodyB.invInertia;
        
        const invEffectiveMass = contact.bodyA.invMass + contact.bodyB.invMass + rotTermA + rotTermB;
        contact.collisionMasses[contactPointIndex] = invEffectiveMass > 0 ? 1.0 / invEffectiveMass : 0;
        contact.collisionMassesCalculated[contactPointIndex] = true;

        // Tangential collision mass
        const rAcrossT = Vector2.cross(rA, contact.tangent);
        const rBcrossT = Vector2.cross(rB, contact.tangent);
        
        const rotTermTA = rAcrossT * rAcrossT * contact.bodyA.invInertia;
        const rotTermTB = rBcrossT * rBcrossT * contact.bodyB.invInertia;
        
        const invEffectiveMassT = contact.bodyA.invMass + contact.bodyB.invMass + rotTermTA + rotTermTB;
        contact.tangentialCollisionMasses[contactPointIndex] = invEffectiveMassT > 0 ? 1.0 / invEffectiveMassT : 0;
        contact.tangentialCollisionMassesCalculated[contactPointIndex] = true;
    }
    
    calculateImpulse(contact, contactPointIndex, velA, velB, angVelA, angVelB, collisionMass, direction, allowNegative = false, restitution = 0.0) {
        const contactPoint = contact.contactPoints[contactPointIndex];
        
        const rA = Vector2.subtract(contactPoint, contact.bodyA.position);
        const rB = Vector2.subtract(contactPoint, contact.bodyB.position);
        
        const velAtContactA = Vector2.add(velA, new Vector2(-rA.y * angVelA, rA.x * angVelA));
        const velAtContactB = Vector2.add(velB, new Vector2(-rB.y * angVelB, rB.x * angVelB));
        
        const relativeVel = Vector2.subtract(velAtContactA, velAtContactB);
        const relativeVelDirection = Vector2.dot(relativeVel, direction);
        
        if (relativeVelDirection < 0 && !allowNegative) {
            return new Vector2(0, 0);
        }
        
        const impulseMagnitude = -(1.0 + restitution) * relativeVelDirection * collisionMass;
        
        return Vector2.multiply(direction, impulseMagnitude);
    }
    
    applyImpulseToVelocities(contact, contactPointIndex, impulse, applyToReal, applyToDelta, applyToResting) {
        const contactPoint = contact.contactPoints[contactPointIndex];
        if (applyToReal) {
            if (!contact.bodyA.isStatic) {
                contact.bodyA.velocity = Vector2.add(contact.bodyA.velocity, Vector2.multiply(impulse, contact.bodyA.invMass));
                const rA = Vector2.subtract(contactPoint, contact.bodyA.position);
                contact.bodyA.angularVelocity += Vector2.cross(rA, impulse) * contact.bodyA.invInertia;
            }
            
            if (!contact.bodyB.isStatic) {
                contact.bodyB.velocity = Vector2.subtract(contact.bodyB.velocity, Vector2.multiply(impulse, contact.bodyB.invMass));
                const rB = Vector2.subtract(contactPoint, contact.bodyB.position);
                contact.bodyB.angularVelocity -= Vector2.cross(rB, impulse) * contact.bodyB.invInertia;
            }
        }
        
        if (applyToDelta) {
            if (!contact.bodyA.isStatic) {
                contact.bodyA.deltaVelocity = Vector2.add(contact.bodyA.deltaVelocity, Vector2.multiply(impulse, contact.bodyA.invMass));
                const rA = Vector2.subtract(contactPoint, contact.bodyA.position);
                contact.bodyA.deltaAngularVelocity += Vector2.cross(rA, impulse) * contact.bodyA.invInertia;
            }
            
            if (!contact.bodyB.isStatic) {
                contact.bodyB.deltaVelocity = Vector2.subtract(contact.bodyB.deltaVelocity, Vector2.multiply(impulse, contact.bodyB.invMass));
                const rB = Vector2.subtract(contactPoint, contact.bodyB.position);
                contact.bodyB.deltaAngularVelocity -= Vector2.cross(rB, impulse) * contact.bodyB.invInertia;
            }
        }
        if (applyToResting) {
            const cache = this.getContactCache(contact.bodyA, contact.bodyB);
            
            if (!contact.bodyA.isStatic) {
                const deltaVA = Vector2.multiply(impulse, contact.bodyA.invMass);
                cache.restingVelocityA = Vector2.add(cache.restingVelocityA, deltaVA);
                const rA = Vector2.subtract(contactPoint, contact.bodyA.position);
                cache.restingAngularVelocityA += Vector2.cross(rA, impulse) * contact.bodyA.invInertia;
            }
            
            if (!contact.bodyB.isStatic) {
                const deltaVB = Vector2.multiply(impulse, contact.bodyB.invMass);
                cache.restingVelocityB = Vector2.subtract(cache.restingVelocityB, deltaVB);
                const rB = Vector2.subtract(contactPoint, contact.bodyB.position);
                cache.restingAngularVelocityB -= Vector2.cross(rB, impulse) * contact.bodyB.invInertia;
            }
        }
    }

    getContactCache(bodyA, bodyB) {
        const key = new ContactCache(bodyA, bodyB).key;
        return this.contactCache.get(key);
    }
    
    cleanupContactCache() {
        for (const [key, cache] of this.contactCache.entries()) {
            if (this.currentTime - cache.lastUpdateTime > this.cacheTimeout) {
                this.contactCache.delete(key);
            }
        }
    }

    // Mouse interaction methods
    handleMouseDown(x, y) {
        this.isMouseDown = true;
        this.mousePosition = new Vector2(x, y);
        
        // Find the body under the mouse cursor
        const bodyUnderMouse = this.getBodyAtPoint(this.mousePosition);
        if (bodyUnderMouse && !bodyUnderMouse.isStatic) {
            this.grabbedBody = bodyUnderMouse;
            // Calculate offset from object center to mouse position
            this.grabOffset = Vector2.subtract(this.mousePosition, bodyUnderMouse.position);
        }
    }
    
    handleMouseMove(x, y) {
        this.mousePosition = new Vector2(x, y);
    }
    
    handleMouseUp() {
        this.isMouseDown = false;
        this.grabbedBody = null;
        this.grabOffset = new Vector2(0, 0);
    }
    
    getBodyAtPoint(point) {
        // Check bodies in reverse order (top to bottom rendering)
        for (let i = this.bodies.length - 1; i >= 0; i--) {
            const body = this.bodies[i];
            const vertices = body.getVertices();
            if (this.isPointInRectangle(point, vertices)) {
                return body;
            }
        }
        return null;
    }
    
    updateMouseInteraction() {
        if (this.grabbedBody && this.isMouseDown) {
            // Calculate target position (mouse position minus grab offset)
            const targetPosition = Vector2.subtract(this.mousePosition, this.grabOffset);
            
            // Move object position towards target by 10%
            const positionDelta = Vector2.subtract(targetPosition, this.grabbedBody.position);
            const newPosition = Vector2.add(this.grabbedBody.position, Vector2.multiply(positionDelta, 0.1));
            this.grabbedBody.position = newPosition;
            
            // Reduce velocity by 40%
            this.grabbedBody.velocity = Vector2.multiply(this.grabbedBody.velocity, 0.6);
            this.grabbedBody.angularVelocity *= 0.6;
        }
    }

    applyRestingDecay() {
        for (const contact of this.contacts) {
            const cache = this.getContactCache(contact.bodyA, contact.bodyB);

            let ff = 1;
            cache.restingVelocityA = Vector2.multiply(cache.restingVelocityA, ff);
            cache.restingVelocityB = Vector2.multiply(cache.restingVelocityB, ff);
            cache.restingAngularVelocityA *= ff;
            cache.restingAngularVelocityA *= ff;
        }
    }
    
    render(ctx) {
        ctx.clearRect(0, 0, this.worldWidth, this.worldHeight);
        
        // Render bodies
        for (const body of this.bodies) {
            this.renderRectangle(ctx, body);
        }
        
        // Render contact points
        ctx.fillStyle = 'red';
        for (const contact of this.contacts) {
            for (const contactPoint of contact.contactPoints) {
                ctx.beginPath();
                ctx.arc(contactPoint.x, contactPoint.y, 3, 0, 2 * Math.PI);
                ctx.fill();
            }
        }
        // Render grab indicator
        if (this.grabbedBody && this.isMouseDown) {
            ctx.strokeStyle = 'lime';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(this.grabbedBody.position.x, this.grabbedBody.position.y, Math.max(this.grabbedBody.width, this.grabbedBody.height) / 2 + 5, 0, 2 * Math.PI);
            ctx.stroke();
        }
    }
    
    renderRectangle(ctx, rect) {
        ctx.save();
        ctx.translate(rect.position.x, rect.position.y);
        ctx.rotate(rect.angle);
        
        ctx.fillStyle = rect.color;
        ctx.fillRect(-rect.width / 2, -rect.height / 2, rect.width, rect.height);
        
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1;
        ctx.strokeRect(-rect.width / 2, -rect.height / 2, rect.width, rect.height);
        
        // Draw velocity vector
        if (!rect.isStatic) {
            ctx.strokeStyle = 'yellow';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, 0);
            const velScale = 0.1;
            ctx.lineTo(rect.velocity.x * velScale, rect.velocity.y * velScale);
            ctx.stroke();
        }
        
        ctx.restore();
    }
    
    // Utility methods
    getBodies() {
        return this.bodies;
    }
    
    getActiveContactCount() {
        return this.contacts.length;
    }
    
    reset() {
        this.bodies = [];
        this.contacts = [];
        this.contactCache.clear();
        this.currentTime = 0;
        this.initializeWorld();
    }
    
    togglePause() {
        this.isPaused = !this.isPaused;
    }
}