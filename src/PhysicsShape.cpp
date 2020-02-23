#include "PhysicsShape.h"

using namespace basicgraphics;
using namespace std;
using namespace glm;

// For efficiency we'll use the same collision box shape for all Shape objects
btCollisionShape* PhysicsShape::COLLISIONSHAPE =  new btBoxShape(btVector3(0.5f,0.5f,0.5f));

PhysicsShape::PhysicsShape(mat4 &transform, float startingMass, btDiscreteDynamicsWorld* dynamicsWorld) :
_dynamicsWorld(dynamicsWorld), _color(1.0), _pickedConstraint(0)
{
    _isConstraintAdded = false;
    
    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setFromOpenGLMatrix(glm::value_ptr(transform));
    
    btScalar mass(startingMass);
    
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0, 0, 0);
    if (isDynamic){
        COLLISIONSHAPE->calculateLocalInertia(mass, localInertia);
    }
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, COLLISIONSHAPE, localInertia);
    _rigidBody = new btRigidBody(rbInfo);
    
    _dynamicsWorld->addRigidBody(_rigidBody);
}
PhysicsShape::~PhysicsShape()
{
    if (_rigidBody && _rigidBody->getMotionState())
    {
        delete _rigidBody->getMotionState();
    }
    _dynamicsWorld->removeCollisionObject(_rigidBody);
}

void PhysicsShape::setDiffuseColor(vec3 color)
{
    _color = color;
}

vec3 PhysicsShape::getDiffuseColor() const {
	return _color;
}

mat4 PhysicsShape::getTransform() const {
    btTransform trans;
    if (_rigidBody && _rigidBody->getMotionState())
    {
        _rigidBody->getMotionState()->getWorldTransform(trans);
    }
    
    float modelMatArray[16];
    trans.getOpenGLMatrix(modelMatArray);
    mat4 modelMatrix = glm::make_mat4(modelMatArray);
    
	return modelMatrix;
}

vec3 PhysicsShape::getPosition() const {
    btTransform trans;
    if (_rigidBody && _rigidBody->getMotionState())
    {
        _rigidBody->getMotionState()->getWorldTransform(trans);
    }
    btVector3 origin = trans.getOrigin();
    
    return vec3(origin.getX(), origin.getY(), origin.getZ());
}

void PhysicsShape::setTransform(glm::mat4 transform)
{
    if (_isConstraintAdded && _pickedConstraint) {
        
        btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*>(_pickedConstraint);
        if (pickCon)
        {
            btVector3 newPivotB;
            
            vec3 pos = vec3(column(transform, 3));
            
            newPivotB.setX(pos.x);
            newPivotB.setY(pos.y);
            newPivotB.setZ(pos.z);
            
            pickCon->getFrameOffsetA().setOrigin(newPivotB);
        }
    }
}
void PhysicsShape::addPositionConstraint(glm::mat4 transform)
{
    if (!_isConstraintAdded){
        _isConstraintAdded = true;
        
        _savedActivationState = _rigidBody->getActivationState();
        _rigidBody->setActivationState(DISABLE_DEACTIVATION);
        
        vec3 pos = vec3(column(transform, 3));
        btVector3 point(pos.x, pos.y, pos.z);
        btVector3 localPivot = _rigidBody->getCenterOfMassTransform().inverse() * point;
        
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(localPivot);
        btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*_rigidBody, tr, false);
        dof6->setLinearLowerLimit(btVector3(0,0,0));
        dof6->setLinearUpperLimit(btVector3(0,0,0));
        dof6->setAngularLowerLimit(btVector3(0,0,0));
        dof6->setAngularUpperLimit(btVector3(0,0,0));
        
        _dynamicsWorld->addConstraint(dof6);
        _pickedConstraint = dof6;
        
        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,0);
        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,1);
        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,2);
        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,3);
        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,4);
        dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,5);
        
        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,0);
        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,1);
        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,2);
        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,3);
        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,4);
        dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,5);
    }
}
void PhysicsShape::removePositionConstraint()
{
    _isConstraintAdded = false;
    if (_pickedConstraint)
    {
        _rigidBody->forceActivationState(_savedActivationState);
        _rigidBody->activate();
        _dynamicsWorld->removeConstraint(_pickedConstraint);
        delete _pickedConstraint;
        _pickedConstraint = 0;
    }
}

