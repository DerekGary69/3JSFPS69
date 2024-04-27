async function AmmoPhysics() {

	if ( 'Ammo' in window === false ) {

		console.error( 'AmmoPhysics: Couldn\'t find Ammo.js' );
		return;

    }

	const AmmoLib = await Ammo(); // eslint-disable-line no-undef

	let frameRate = 144;

	const collisionConfiguration = new AmmoLib.btDefaultCollisionConfiguration();
	const dispatcher = new AmmoLib.btCollisionDispatcher( collisionConfiguration );
	const broadphase = new AmmoLib.btDbvtBroadphase();
	const solver = new AmmoLib.btSequentialImpulseConstraintSolver();
	const world = new AmmoLib.btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );
	world.setGravity( new AmmoLib.btVector3( 0, - 9.8, 0 ) );

	const worldTransform = new AmmoLib.btTransform();

	//

	function getShape( geometry ) {

		const parameters = geometry.parameters;

		// TODO change type to is*

		if ( geometry.type === 'BoxGeometry' ) {

			const sx = parameters.width !== undefined ? parameters.width / 2 : 0.5;
			const sy = parameters.height !== undefined ? parameters.height / 2 : 0.5;
			const sz = parameters.depth !== undefined ? parameters.depth / 2 : 0.5;

			const shape = new AmmoLib.btBoxShape( new AmmoLib.btVector3( sx, sy, sz ) );
			shape.setMargin( 0.05 );

			return shape;

		} else if ( geometry.type === 'SphereGeometry' || geometry.type === 'IcosahedronGeometry' ) {

			const radius = parameters.radius !== undefined ? parameters.radius : 1;

			const shape = new AmmoLib.btSphereShape( radius );
			shape.setMargin( 0.05 );

			return shape;

		}

		return null;

	}


	const meshes = [];
	const meshMap = new WeakMap();

	function addScene( scene ) {

		scene.traverse( function ( child ) {

			if ( child.isMesh ) {

				const physics = child.userData.physics;

				if ( physics ) {

					addMesh( child, physics.mass );

				}

			}

		} );

	}

	function addMesh( mesh, mass = 0 ) {

		const shape = getShape( mesh.geometry );

		if ( shape !== null ) {

			if ( mesh.isInstancedMesh ) {

				handleInstancedMesh( mesh, mass, shape );

			} else if ( mesh.isMesh ) {

				handleMesh( mesh, mass, shape );

			}

		}

	}

	function handleMesh( mesh, mass, shape, offset = { x: 0, y: 0, z: 0 }) {

		const position = mesh.position;
		const quaternion = mesh.quaternion;

		const transform = new AmmoLib.btTransform();
		transform.setIdentity();
		transform.setOrigin( new AmmoLib.btVector3( position.x + offset.x, position.y + offset.y, position.z + offset.z ) );
		transform.setRotation( new AmmoLib.btQuaternion( quaternion.x, quaternion.y, quaternion.z, quaternion.w ) );

		const motionState = new AmmoLib.btDefaultMotionState( transform );

		// console.log(transform)

		const localInertia = new AmmoLib.btVector3( 0, 0, 0 );
		shape.calculateLocalInertia( mass, localInertia );

		const rbInfo = new AmmoLib.btRigidBodyConstructionInfo( mass, motionState, shape, localInertia );

		const body = new AmmoLib.btRigidBody( rbInfo );
		// body.setFriction( 4 );
		world.addRigidBody( body );

		if ( mass > 0 ) {

			meshes.push( mesh );
			meshMap.set( mesh, body );

		}


	}

	function handleInstancedMesh( mesh, mass, shape ) {

		const array = mesh.instanceMatrix.array;

		const bodies = [];

		for ( let i = 0; i < mesh.count; i ++ ) {

			const index = i * 16;

			const transform = new AmmoLib.btTransform();
			transform.setFromOpenGLMatrix( array.slice( index, index + 16 ) );

			const motionState = new AmmoLib.btDefaultMotionState( transform );

			const localInertia = new AmmoLib.btVector3( 0, 0, 0 );
			shape.calculateLocalInertia( mass, localInertia );

			const rbInfo = new AmmoLib.btRigidBodyConstructionInfo( mass, motionState, shape, localInertia );

			const body = new AmmoLib.btRigidBody( rbInfo );
			world.addRigidBody( body );

			bodies.push( body );

		}

		if ( mass > 0 ) {

			meshes.push( mesh );

			meshMap.set( mesh, bodies );

		}

	}

	//

	function setMeshPosition( mesh, position, index = 0 ) {

		if ( mesh.isInstancedMesh ) {

			const bodies = meshMap.get( mesh );
			const body = bodies[ index ];

			body.setAngularVelocity( new AmmoLib.btVector3( 0, 0, 0 ) );
			body.setLinearVelocity( new AmmoLib.btVector3( 0, 0, 0 ) );

			worldTransform.setIdentity();
			worldTransform.setOrigin( new AmmoLib.btVector3( position.x, position.y, position.z ) );
			body.setWorldTransform( worldTransform );

		} else if ( mesh.isMesh ) {

			const body = meshMap.get( mesh );

			body.setAngularVelocity( new AmmoLib.btVector3( 0, 0, 0 ) );
			body.setLinearVelocity( new AmmoLib.btVector3( 0, 0, 0 ) );

			worldTransform.setIdentity();
			worldTransform.setOrigin( new AmmoLib.btVector3( position.x, position.y, position.z ) );
			body.setWorldTransform( worldTransform );

		}

	}

    //

    function applyVelocity(mesh, velocity, index = 0, damping = .25, multiplier = 1, isJumping = false) {
        if (mesh.isInstancedMesh) {
            const bodies = meshMap.get(mesh);
            const body = bodies[index];
            const currentVelocity = body.getLinearVelocity();
            const newVelocity = new AmmoLib.btVector3(
                currentVelocity.x() * damping + (velocity.x * multiplier),
                currentVelocity.y() * damping + (velocity.y * multiplier),
                currentVelocity.z() * damping + (velocity.z * multiplier)
            );
            body.setLinearVelocity(newVelocity);
        } else if (mesh.isMesh) {
            const body = meshMap.get(mesh);
            const currentVelocity = body.getLinearVelocity();

			let velocityIfNotJumping = new AmmoLib.btVector3(velocity.x, currentVelocity.y(), velocity.z);

            const newVelocity = new AmmoLib.btVector3(
                (currentVelocity.x() * damping) + (velocity.x * multiplier),
                (isJumping ? currentVelocity.y() * damping + (velocity.y * multiplier) : velocityIfNotJumping.y()),
                (currentVelocity.z() * damping) + (velocity.z * multiplier)
            );
            body.setLinearVelocity(newVelocity);
        }
    }

	//

	//adds rotation force to the mesh until it reaches the target rotation
	function applyRotation(mesh, targetRotation, index = 0, damping = .25) {
		if (mesh.isInstancedMesh) {
			const bodies = meshMap.get(mesh);
			const body = bodies[index];
			const currentRotation = body.getAngularVelocity();
			const newRotation = new AmmoLib.btVector3(
				currentRotation.x() * damping + targetRotation.x,
				currentRotation.y() * damping + targetRotation.y,
				currentRotation.z() * damping + targetRotation.z
			);
			body.setAngularVelocity(newRotation);
		}
		else if (mesh.isMesh) {
			const body = meshMap.get(mesh);
			const currentRotation = body.getAngularVelocity();
			const newRotation = new AmmoLib.btVector3(
				currentRotation.x() * damping + targetRotation.x,
				currentRotation.y() * damping + targetRotation.y,
				currentRotation.z() * damping + targetRotation.z
			);
			body.setAngularVelocity(newRotation);
		}
	}

    //

    function setRotation(mesh, quaternion, index = 0) {
       
        if (mesh.isInstancedMesh) {
            const bodies = meshMap.get(mesh);
            const body = bodies[index];

            worldTransform.setIdentity();
            worldTransform.setRotation(new AmmoLib.btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));
            body.setWorldTransform(worldTransform);

        } else if (mesh.isMesh) {
            const body = meshMap.get(mesh);

            worldTransform.setIdentity();
            worldTransform.setRotation(new AmmoLib.btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));
            body.setWorldTransform(worldTransform);
        }
    }

    //

    function lockRotation(mesh, lock, index = 0) {
        if (mesh.isInstancedMesh) {
            const bodies = meshMap.get(mesh);
            const body = bodies[index];
            body.setAngularFactor(new AmmoLib.btVector3(lock.x, lock.y, lock.z));
        } else if (mesh.isMesh) {
            const body = meshMap.get(mesh);
            body.setAngularFactor(new AmmoLib.btVector3(lock.x, lock.y, lock.z));
        }
    }

	//

	function createConvexHullShape(vertices, mass, mesh, position, scale, quaternion) {

		const shape = new AmmoLib.btConvexHullShape();

		for (let i = 0; i < vertices.length; i++) {
			const vertex = vertices[i];
			const vec = new AmmoLib.btVector3(vertex.x, vertex.y, vertex.z);
			shape.addPoint(vec);
		}

		
		
		// mesh.position.x = position.x;
		// mesh.position.y = position.y;
		// mesh.position.z = position.z;

		// shape.setMargin(5);
		shape.setLocalScaling(new AmmoLib.btVector3(scale, scale, scale));
		shape.setMargin(-0.01);
		mesh.scale.set(scale, scale, scale);
		mesh.position.set(position.x, position.y, position.z);
		mesh.quaternion.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
		// console.log(position);
		

		handleMesh(mesh, mass, shape);
		// setMeshPosition(mesh, position);
		return mesh;
	}

	function createBvhTriangleMeshShape(vertices, mesh, position, scale, quaternion) {
		const triangleMesh = new AmmoLib.btTriangleMesh();

		for (let i = 0; i < vertices.length; i += 3) {
			const vertex1 = vertices[i];
			const vertex2 = vertices[i + 1];
			const vertex3 = vertices[i + 2];
		
			if (vertex1 && vertex2 && vertex3) {
				const vec1 = new AmmoLib.btVector3(vertex1.x, vertex1.y, vertex1.z);
				const vec2 = new AmmoLib.btVector3(vertex2.x, vertex2.y, vertex2.z);
				const vec3 = new AmmoLib.btVector3(vertex3.x, vertex3.y, vertex3.z);
		
				triangleMesh.addTriangle(vec1, vec2, vec3);
			}
		}
		
		const shape = new AmmoLib.btBvhTriangleMeshShape(triangleMesh, true, true);

		shape.setLocalScaling(new AmmoLib.btVector3(scale.x, scale.y, scale.z));
		shape.setMargin(.01);
		mesh.scale.set(scale.x, scale.y, scale.z);
		mesh.position.set(position.x, position.y, position.z);
		mesh.rotation.set(0, 0, 0);
		mesh.quaternion.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
		

		handleMesh(mesh, 0, shape);
		return mesh;
	}

	function createCompoundShapeFromVertices(vertices, mass, mesh, position, scale, quaternion) {
		const compoundShape = new AmmoLib.btCompoundShape();

		for (let i = 0; i < vertices.length; i += 3) {
			const vertex1 = vertices[i];
			const vertex2 = vertices[i + 1];
			const vertex3 = vertices[i + 2];

			if (vertex1 && vertex2 && vertex3) {
				// Create a triangle shape
				const triangleShape = new AmmoLib.btTriangleShape(
					new AmmoLib.btVector3(vertex1.x, vertex1.y, vertex1.z),
					new AmmoLib.btVector3(vertex2.x, vertex2.y, vertex2.z),
					new AmmoLib.btVector3(vertex3.x, vertex3.y, vertex3.z)
				);

				// Create a transform for the triangle shape
				const transform = new AmmoLib.btTransform();
				transform.setIdentity();
				transform.setOrigin(new AmmoLib.btVector3(0, 0, 0));  // Adjust this as needed

				// Add the triangle shape to the compound shape
				compoundShape.addChildShape(transform, triangleShape);
			}
		}

		compoundShape.setLocalScaling(new AmmoLib.btVector3(scale.x, scale.y, scale.z));
		compoundShape.setMargin(-0.01);
		mesh.scale.set(scale.x, scale.y, scale.z);
		mesh.position.set(position.x, position.y, position.z);
		mesh.rotation.set(0, 0, 0);
		mesh.quaternion.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

		handleMesh(mesh, mass, compoundShape);
		return mesh;
	}

	//

	let lastTime = 0;

	function step() {

		const time = performance.now();

		if ( lastTime > 0 ) {

			const delta = ( time - lastTime ) / 1000;

			world.stepSimulation( delta, 10 );

			//

			for ( let i = 0, l = meshes.length; i < l; i ++ ) {

				const mesh = meshes[ i ];

              

				if ( mesh.isInstancedMesh ) {

					const array = mesh.instanceMatrix.array;
					const bodies = meshMap.get( mesh );

					for ( let j = 0; j < bodies.length; j ++ ) {

						const body = bodies[ j ];

						const motionState = body.getMotionState();
						motionState.getWorldTransform( worldTransform );

						const position = worldTransform.getOrigin();
						let quaternion = worldTransform.getRotation();

						compose( position, quaternion, array, j * 16 );

					}

					mesh.instanceMatrix.needsUpdate = true;
					mesh.computeBoundingSphere();

				} else if ( mesh.isMesh ) {

					const body = meshMap.get( mesh );

					const motionState = body.getMotionState();
					motionState.getWorldTransform( worldTransform );

					const position = worldTransform.getOrigin();

                    
					let quaternion = worldTransform.getRotation();
					mesh.position.set( position.x(), position.y(), position.z() );

                    mesh.quaternion.set( quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w() );


					

				}

			}

		}

		lastTime = time;

	}

	// animate

	setInterval( step, 1000 / frameRate );

	return {
		addScene: addScene,
		addMesh: addMesh,
		setMeshPosition: setMeshPosition,
        applyVelocity: applyVelocity,
        setRotation: setRotation,
        lockRotation: lockRotation,
		applyRotation: applyRotation,
		meshMap: meshMap,
		step: step,
		frameRate: frameRate,
		createConvexHullShape: createConvexHullShape,
		createBvhTriangleMeshShape: createBvhTriangleMeshShape,
		createCompoundShapeFromVertices: createCompoundShapeFromVertices


		// addCompoundMesh
	};

}

function compose( position, quaternion, array, index ) {

	const x = quaternion.x(), y = quaternion.y(), z = quaternion.z(), w = quaternion.w();
	const x2 = x + x, y2 = y + y, z2 = z + z;
	const xx = x * x2, xy = x * y2, xz = x * z2;
	const yy = y * y2, yz = y * z2, zz = z * z2;
	const wx = w * x2, wy = w * y2, wz = w * z2;

	array[ index + 0 ] = ( 1 - ( yy + zz ) );
	array[ index + 1 ] = ( xy + wz );
	array[ index + 2 ] = ( xz - wy );
	array[ index + 3 ] = 0;

	array[ index + 4 ] = ( xy - wz );
	array[ index + 5 ] = ( 1 - ( xx + zz ) );
	array[ index + 6 ] = ( yz + wx );
	array[ index + 7 ] = 0;

	array[ index + 8 ] = ( xz + wy );
	array[ index + 9 ] = ( yz - wx );
	array[ index + 10 ] = ( 1 - ( xx + yy ) );
	array[ index + 11 ] = 0;

	array[ index + 12 ] = position.x();
	array[ index + 13 ] = position.y();
	array[ index + 14 ] = position.z();
	array[ index + 15 ] = 1;

}

export { AmmoPhysics };
