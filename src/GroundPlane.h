//
//  GroundPlane.hpp
//  
//
//  Created by Bret Jackson on 2/15/17.
//
//

#ifndef GroundPlane_hpp
#define GroundPlane_hpp

#include <stdio.h>

#include <Mesh.h>
#include <GLSLProgram.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <btBulletDynamicsCommon.h>
    
class GroundPlane {
    public:
        
        /*!
         * Creates a plane with the specified normal with running through the point
         */
        GroundPlane(const glm::vec3 &point, const glm::vec3 &normal, btDiscreteDynamicsWorld* dynamicsWorld);
        virtual ~GroundPlane();
        
    virtual void draw(basicgraphics::GLSLProgram &shader);

        
    protected:
        std::unique_ptr<basicgraphics::Mesh> _mesh;
        const glm::vec3 _point;
        glm::vec3 _normal;
    
        btRigidBody* _rigidBody;
        btCollisionShape* _colShape;
        btDiscreteDynamicsWorld* _dynamicsWorld;
};


#endif /* GroundPlane_hpp */
