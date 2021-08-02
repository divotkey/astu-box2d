/*
 * ASTU/Box2D
 * An integration of Erin Catto's 2D Physics Engine to AST-Utilities.
 * 
 * Copyright (c) 2020, 2021 Roman Divotkey. All rights reserved.
 */

// Local includes
#include "CBox2DColliders.h"

// Box2D includes
#include <box2d/b2_circle_shape.h>
#include <box2d/b2_polygon_shape.h>

namespace astu::suite2d {

    void CBox2DCircleCollider::CreateFixture(b2Body & body)
    {
        b2FixtureDef fixtureDef;

        ConfigureFixtureDef(fixtureDef);
		b2CircleShape shape;
		shape.m_radius = GetRadius();
        shape.m_p.Set(GetOffset().x, GetOffset().y);

        fixtureDef.shape = &shape;
        fixture = body.CreateFixture(&fixtureDef);
    }

    std::vector<b2Vec2> CBox2DPolygonCollider::tempVertices;

    void CBox2DPolygonCollider::CreateFixture(b2Body & body)
    {
        b2FixtureDef fixtureDef;

        ConfigureFixtureDef(fixtureDef);

        tempVertices.clear();
        for (auto vtx : polygon->GetVertices()) {
            tempVertices.push_back(
                    b2Vec2(vtx.x + GetOffset().x, vtx.y + GetOffset().y));
        }

        b2PolygonShape shape;
        shape.Set(tempVertices.data(), static_cast<int32>(tempVertices.size()));
        fixtureDef.shape = &shape;

        fixture = body.CreateFixture(&fixtureDef);
    }


} // end of namespace
