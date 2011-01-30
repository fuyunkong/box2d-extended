/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <Box2D/Dynamics/Controllers/b2RelativeForceController.h>

b2RelativeForceController::b2RelativeForceController(const b2RelativeForceControllerDef* def) : b2Controller(def)
{
	force = def->force;
	position = def->position;
	innerRadius = def->innerRadius;
	outerRadius = def->outerRadius;
	constant = def->constant;
	escape = def->escape;
}

void b2RelativeForceController::Step(const b2TimeStep& step)
{
	for(b2ControllerEdge *i=m_bodyList;i;i=i->nextBody){
		b2Body* body = i->body;
		if(!body->IsAwake())
			continue;
		b2Vec2 delta = body->GetPosition() - position;
		float32 dist = delta.Length();
		if (dist > outerRadius) {
			continue;
		}
		if (dist < innerRadius) {
			if (escape) {
				continue;
			}
			body->SetLinearVelocity(b2Vec2(0.0f, 0.0f));
			continue;
		}
		// normalize the vector
		delta *= (1 / dist);
		delta *= force * ((outerRadius - innerRadius) - (dist - innerRadius));
		body->ApplyForce(delta, body->GetPosition());
	}
}

void b2RelativeForceController::Destroy(b2BlockAllocator* allocator)
{
	allocator->Free(this, sizeof(b2RelativeForceController));
}


b2RelativeForceController* b2RelativeForceControllerDef::Create(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2RelativeForceController));
	return new (mem) b2RelativeForceController(this);
}
