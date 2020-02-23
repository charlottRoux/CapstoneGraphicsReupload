#ifndef SHAPE_H
#define SHAPE_H

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <BasicGraphics.h>

#include <iostream>
#include <sstream>
#include <fstream>

#include <btBulletDynamicsCommon.h>

/** This class represents a rigid body physics object. It's defined by a collider shape
	(a box). The shape also keeps track of the current transformation of the object and the color
*/
class PhysicsShape {
public:

    // Constructor creates the shape based on the transform. A starting mass > 0 means the shape
    // responds to gravity. 1.0 is a good default value.
    PhysicsShape(glm::mat4 &transform, float startingMass, btDiscreteDynamicsWorld* dynamicsWorld);
    virtual ~PhysicsShape();
    
    
	mat4 getTransform() const;
    glm::vec3 getPosition() const;

	void setDiffuseColor(vec3 color);
	vec3 getDiffuseColor() const;
    
	// These method enable to you add a rigid constraint to the shape. 
	// Calling addPositionConstraint locks the shape to the given transform.
	// This transform can be updated each frame by calling setTransform.
	// Finally, calling removePositionConstraint will set the object back
	// to responding to gravity and normal physics motion.
    void addPositionConstraint(glm::mat4 transform);
	void setTransform(glm::mat4 transform);
    void removePositionConstraint();

       
private:

	static btCollisionShape* COLLISIONSHAPE;

    btRigidBody* _rigidBody;
    btDiscreteDynamicsWorld* _dynamicsWorld;
    btTypedConstraint* _pickedConstraint;
    int _savedActivationState;
    
    vec3 _color;
    bool _isConstraintAdded;
    
};
#endif //SHAPE_H
