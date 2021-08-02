/*
 * ASTU/Box2D
 * An integration of Erin Catto's 2D Physics Engine to AST-Utilities.
 * 
 * Copyright (c) 2020, 2021 Roman Divotkey. All rights reserved.
 */

// Local includes
#include "CBox2DBody.h"

// Box2D includes
#include <box2d/box2d.h>

using namespace std;

namespace astu::suite2d {

    void CBox2DBody::SetType(CBody::Type bodyType)
    {
        CBody::SetType(bodyType);

        if (boxBody) {
            switch(bodyType) {
                
            case CBody::Type::Static:
                boxBody->SetType(b2BodyType::b2_staticBody);
                break;

            case CBody::Type::Kinematic:
                boxBody->SetType(b2BodyType::b2_kinematicBody);
                break;

            case CBody::Type::Dynamic:
                boxBody->SetType(b2BodyType::b2_dynamicBody);
                break;
            }
        }
    }

    Vector2f CBox2DBody::GetLinearVelocity() const
    {
        if (boxBody) {
            const b2Vec2& v = boxBody->GetLinearVelocity();
            return Vector2f(v.x, v.y);
        }
        return CBody::GetLinearVelocity();
    }

    CBody& CBox2DBody::SetLinearVelocity(float vx, float vy)
    {
        CBody::SetLinearVelocity(vx, vy);
        if (boxBody) {
            boxBody->SetLinearVelocity(b2Vec2(vx, vy));
        }
        return *this;
    }

    float CBox2DBody::GetAngularVelocity() const
    {
        if (boxBody) {
            return boxBody->GetAngularVelocity();
        }
        return CBody::GetAngularVelocity();
    }

    CBody& CBox2DBody::SetAngularVelocity(float av) 
    {
        CBody::SetAngularVelocity(av);
        if (boxBody) {
            boxBody->SetAngularVelocity(av);
        }

        return *this;
    }

    void CBox2DBody::SetLinearDamping(float damping)
    {
        CBody::SetLinearDamping(damping);

        if (boxBody) {
            boxBody->SetLinearDamping(damping);
        }
    }

    void CBox2DBody::SetAngularDamping(float damping)
    {
        CBody::SetAngularDamping(damping);

        if (boxBody) {
            boxBody->SetAngularDamping(damping);
        }
    }

    void CBox2DBody::ApplyTorque(float torque)
    {
        if (boxBody) {
            boxBody->ApplyTorque(torque, true);
        }
    }

    Vector2f CBox2DBody::GetWorldVector(float lvx, float lvy)
    {
        if (boxBody) {
            b2Vec2 wv = boxBody->GetWorldVector(b2Vec2(lvx, lvy));
            return Vector2f(wv.x, wv.y);
        }
        // Difficult to decide what to do. Should be throw an exception instead?
        return Vector2f::Zero;
    }

    Vector2f CBox2DBody::GetWorldPoint(float lpx, float lpy)
    {
        if (boxBody) {
            b2Vec2 wp = boxBody->GetWorldPoint(b2Vec2(lpx, lpy));
            return Vector2f(wp.x, wp.y);
        }
        // Difficult to decide what to do. Should be throw an exception instead?
        return Vector2f::Zero;
    }

    Vector2f CBox2DBody::GetLocalVector(float wvx, float wvy)
    {
        if (boxBody) {
            b2Vec2 lv = boxBody->GetLocalVector(b2Vec2(wvx, wvy));
            return Vector2f(lv.x, lv.y);
        }
        // Difficult to decide what to do. Should be throw an exception instead?
        return Vector2f::Zero;
    }

    Vector2f CBox2DBody::GetLocalPoint(float wpx, float wpy)
    {
        if (boxBody) {
            b2Vec2 lp = boxBody->GetLocalPoint(b2Vec2(wpx, wpy));
            return Vector2f(lp.x, lp.y);
        }
        // Difficult to decide what to do. Should be throw an exception instead?
        return Vector2f::Zero;
    }

    void CBox2DBody::ApplyForce(const Vector2f& force)
    {
        if (boxBody) {
            boxBody->ApplyForceToCenter(b2Vec2(force.x, force.y), true);
        }
    }

} // end of namespace