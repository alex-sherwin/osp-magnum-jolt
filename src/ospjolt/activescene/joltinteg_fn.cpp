/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "joltinteg_fn.h"          // IWYU pragma: associated
#include <osp/activescene/basic_fn.h>

#include <utility>                   // for std::exchange
#include <cassert>                   // for assert

// IWYU pragma: no_include <cstddef>
// IWYU pragma: no_include <type_traits>

using namespace ospjolt;

// for the 0xrrggbb_rgbf and angle literals
using namespace Magnum::Math::Literals;

using osp::EShape;

using osp::active::ActiveEnt;
using osp::active::ACtxPhysics;
using osp::active::SysSceneGraph;

using osp::Matrix3;
using osp::Matrix4;
using osp::Vector3;

void SysJolt::resize_body_data(ACtxJoltWorld& rCtxWorld)
{
    std::size_t const capacity = rCtxWorld.m_bodyIds.capacity();
    rCtxWorld.m_ospToJoltBodyId.resize(capacity);
    rCtxWorld.m_bodyToEnt      .resize(capacity);
    rCtxWorld.m_bodyFactors    .resize(capacity);
}


void SysJolt::update_translate(ACtxPhysics& rCtxPhys, ACtxJoltWorld& rCtxWorld) noexcept
{

    // Origin translation
    if (Vector3 const translate = std::exchange(rCtxPhys.m_originTranslate, {});
        ! translate.isZero())
    {
        PhysicsSystem* pJoltWorld = rCtxWorld.m_world.get();
        BodyIDVector allBodiesIds;
        pJoltWorld->GetBodies(allBodiesIds);

        BodyInterface &body_interface = pJoltWorld->GetBodyInterface();

        // Translate every jolt body
        for (JoltBodyId bodyId : allBodiesIds)
        {
            RVec3 position = body_interface.GetPosition(bodyId);
            Matrix4 matrix;
            position += RVec3(translate.x(), translate.y(), translate.z());
            //As we are translating the whole world, we don't need to wake up asleep bodies. 
            body_interface.SetPosition(bodyId, position, EActivation::DontActivate);
        }
    }
}

using Corrade::Containers::ArrayView;

void SysJolt::update_world(
        ACtxPhysics&                rCtxPhys,
        ACtxJoltWorld&              rCtxWorld,
        float                       timestep,
        ACtxSceneGraph const&       rScnGraph,
        ACompTransformStorage_t&    rTf) noexcept
{
    PhysicsSystem *pJoltWorld = rCtxWorld.m_world.get();
    BodyInterface &body_interface = pJoltWorld->GetBodyInterface();

    // Apply changed velocities
    for (auto const& [ent, vel] : std::exchange(rCtxPhys.m_setVelocity, {}))
    {
        OspBodyId const ospBodyId     = rCtxWorld.m_entToBody.at(ent);
        JoltBodyId const bodyId     = rCtxWorld.m_ospToJoltBodyId[ospBodyId];

        body_interface.SetLinearVelocity(bodyId, Vec3(vel.x(), vel.y(), vel.z()));
    }

    rCtxWorld.m_pTransform = std::addressof(rTf);

    // Update the world
    pJoltWorld->Update(timestep, 1, &rCtxWorld.m_temp_allocator, rCtxWorld.m_joltJobSystem.get()); //TODO : job system
}

void SysJolt::remove_components(ACtxJoltWorld& rCtxWorld, ActiveEnt ent) noexcept
{
    auto itBodyId = rCtxWorld.m_entToBody.find(ent);

    if (itBodyId != rCtxWorld.m_entToBody.end())
    {
        OspBodyId const bodyId = itBodyId->second;
        rCtxWorld.m_bodyToEnt[bodyId] = lgrn::id_null<ActiveEnt>();
        rCtxWorld.m_entToBody.erase(itBodyId);
    }

}

TransformedShapePtr_t SysJolt::create_primitive(ACtxJoltWorld &rCtxWorld, osp::EShape shape)
{
    Shape* joltShape;
    switch (shape)
    {
    case EShape::Sphere:
        joltShape = (Shape *) new SphereShape(1.0f);
        break;
    case EShape::Box:
        joltShape =  (Shape *) new BoxShape(Vec3Arg(1.0f, 1.0f, 1.0f));
        break;
    case EShape::Cylinder:
        joltShape =  (Shape *) new CylinderShape(1.0f, 2.0f);
        break;
    default:
        // TODO: support other shapes, sphere is used for now
        joltShape =  (Shape *) new SphereShape(1.0f);
        break;
    }
    return TransformedShapePtr_t(new TransformedShape(RVec3::sZero(), Quat::sZero(), joltShape, BodyID(), SubShapeIDCreator()));
}

void SysJolt::orient_shape(TransformedShapePtr_t& pJoltShape, osp::EShape ospShape, osp::Vector3 const &translation, osp::Matrix3 const &rotation, osp::Vector3 const &scale)
{
    auto rawQuat = Magnum::Quaternion::fromMatrix(rotation).data();
    Quat joltRotation(rawQuat[0], rawQuat[1], rawQuat[2], rawQuat[3]);
    
    pJoltShape->SetWorldTransform(RVec3Arg(translation.x(), translation.y(), translation.z()), joltRotation, Vec3Arg(scale.x(), scale.y(), scale.z()));
    
}


void SysJolt::find_shapes_recurse(
        ACtxPhysics const&                      rCtxPhys,
        ACtxJoltWorld&                          rCtxWorld,
        ACtxSceneGraph const&                   rScnGraph,
        ACompTransformStorage_t const&          rTf,
        ActiveEnt                               ent,
        Matrix4 const&                          transform,
        CompoundShapeSettings&                  pCompound) noexcept
{
    // Add jolt shape if exists
    if (rCtxWorld.m_shapes.contains(ent))
    {
        TransformedShapePtr_t& pShape
                = rCtxWorld.m_shapes.get(ent);

        // Set transform relative to root body

        // cylinder needs to be rotated 90 degrees Z to aligned with Y axis
        // TODO: replace this with something more sophisticated some time (and re-enable it maybe)
        // Matrix4 const &colliderTf
        //         = (rCtxPhys.m_shape[ent] != EShape::Cylinder)
        //         ? transform : transform * Matrix4::rotationZ(90.0_degf);

        SysJolt::orient_shape(pShape, rCtxPhys.m_shape[ent], transform.translation(), transform.rotation(), transform.scaling());
        Ref<Shape> rScaledShape = pShape->mShape->ScaleShape(Vec3(pShape->mShapeScale)).Get();
        pCompound.AddShape(pShape->mShapePositionCOM, pShape->mShapeRotation, rScaledShape);
    }

    if ( ! rCtxPhys.m_hasColliders.contains(ent) )
    {
        return;
    }

    // Recurse into children if there are more colliders
    for (ActiveEnt child : SysSceneGraph::children(rScnGraph, ent))
    {
        if (rTf.contains(child))
        {
            ACompTransform const &rChildTransform = rTf.get(child);

            Matrix4 const childMatrix = transform * rChildTransform.m_transform;

            find_shapes_recurse(
                    rCtxPhys, rCtxWorld, rScnGraph, rTf, child, childMatrix, pCompound);
        }

    }

}