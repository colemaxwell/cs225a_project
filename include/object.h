/**
 * @file object.h
 * @brief Helper function to add objects in chai world (without physics)
 * 
 */

#include "Sai2Graphics.h"

#ifndef _OBJECT_H_
#define _OBJECT_H_

void addSphere(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const double radius,
                const Vector4d& rgba) {

    // initialize a cGenericObject to represent this object in the world
    auto object = new chai3d::cMesh();
    chai3d::cCreateSphere(object, radius, 100, 100);  // last two values are resolution
    object->m_name = name;
    // set object position and rotation
    object->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        object->setLocalRot(tmp_cmat3);	
    }

    // visuals
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    object->m_material->setColor(*color);

    // add to world
    graphics->_world->addChild(object);
}

void addBox(Sai2Graphics::Sai2Graphics* graphics,
                const string& name,
                const Vector3d& pos,
                const Quaterniond& ori,
                const Vector3d& dim,
                const Vector4d& rgba) {

    // initialize a cGenericObject to represent this object in the world
    auto object = new chai3d::cMesh();
    chai3d::cCreateBox(object, dim(0), dim(1), dim(2));  // last two values are resolution
    object->m_name = name;
    // set object position and rotation
    object->setLocalPos(chai3d::cVector3d(pos(0), pos(1), pos(2)));
    { // brace temp variables to separate scope
        Quaternion<double> tmp_q(ori.w(), ori.x(), ori.y(), ori.z());
        chai3d::cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
        object->setLocalRot(tmp_cmat3);	
    }

    // visuals
    // chai3d::cColorf* color = NULL;
    auto color = new chai3d::cColorf(rgba(0), rgba(1), rgba(2), rgba(3));
    object->m_material->setColor(*color);

    // add to world
    graphics->_world->addChild(object);
}

#endif