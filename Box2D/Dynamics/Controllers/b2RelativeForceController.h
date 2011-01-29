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

#ifndef B2_RELATIVEFORCECONTROLLER_H
#define B2_RELATIVEFORCECONTROLLER_H

#include <Box2D/Dynamics/Controllers/b2Controller.h>

class b2RelativeForceControllerDef;

/// Applies a relative force every frame based upon a relation to a position
class b2RelativeForceController : public b2Controller {
public:
	/// The maximum force to apply
	float32 force;
	/// The position to apply the force from
	b2Vec2 position;
	/// The distance at which point the force should no longer be applied
	float32 radius;
	/// Is the force constant when bodies are within the radius or is there a dropoff
	bool constant;
	
	/// @see b2Controller::Step
	void Step(const b2TimeStep& step);
	
protected:
	void Destroy(b2BlockAllocator* allocator);
	
private:
	friend class b2RelativeForceControllerDef;
	b2RelativeForceController(const b2RelativeForceControllerDef* def);
	
};

/// This class is used to build constant acceleration controllers
class b2RelativeForceControllerDef : public b2ControllerDef
{
public:
	/// The maximum force to apply
	float32 force;
	/// The position to apply the force from
	b2Vec2 position;
	/// The distance at which point the force should no longer be applied
	float32 radius;
	/// Is the force constant when bodies are within the radius or is there a dropoff
	bool constant;
private:
	b2RelativeForceController* Create(b2BlockAllocator* allocator) const;
};

#endif
