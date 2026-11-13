// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT
// Exact class extracted from Brickwright JoltPhysics.js JoltJS.h at
// 767d41f8c29af65e457947574d2e34f402d98d57; class SHA256 af09c1ce6f66d17816d7905e8eccf8ad6c17abc195c27c41db84b7eb3cad35d5.
#pragma once
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
JPH_NAMESPACE_BEGIN
class PartPairSimShapeFilter : public SimShapeFilter
{
public:
	/// Collider inCollider of inBody (its leaf shape's user data) belongs to part inPart (any number but cNoPart)
	void					SetColliderPart(const BodyID &inBody, uint inCollider, uint inPart)
	{
		Array<uint> &parts = mParts[inBody.GetIndexAndSequenceNumber()];
		if (parts.size() <= inCollider)
			parts.resize(inCollider + 1, cNoPart);
		parts[inCollider] = inPart;
	}

	/// Filter the pair inBody1, inBody2 part by part: the listed part pairs are kept apart, or (inCollideList) only they collide
	void					AddBodyPair(const BodyID &inBody1, const BodyID &inBody2, bool inCollideList)
	{
		bool swap;
		mPairs[sBodyKey(inBody1, inBody2, swap)].mCollideList = inCollideList;
	}

	/// List part inPart1 of inBody1 with part inPart2 of inBody2 (the pair given with AddBodyPair, either order)
	void					AddPartPair(const BodyID &inBody1, const BodyID &inBody2, uint inPart1, uint inPart2)
	{
		bool swap;
		Pair &pair = mPairs[sBodyKey(inBody1, inBody2, swap)];
		pair.mParts.insert(swap? sKey(inPart2, inPart1) : sKey(inPart1, inPart2));
	}

	/// Forget every body, part and pair
	void					Clear()
	{
		mParts.clear();
		mPairs.clear();
	}

	virtual bool			ShouldCollide(const Body &inBody1, const Shape *inShape1, const SubShapeID &, const Body &inBody2, const Shape *inShape2, const SubShapeID &) const override
	{
		bool swap;
		UnorderedMap<uint64, Pair>::const_iterator pair = mPairs.find(sBodyKey(inBody1.GetID(), inBody2.GetID(), swap));
		if (pair == mPairs.end())
			return true;
		// Jolt asks at every level of the hierarchy: a compound passes, its leaves decide
		if (inShape1->GetType() == EShapeType::Compound || inShape2->GetType() == EShapeType::Compound)
			return true;
		uint part1 = sPartOf(mParts, inBody1.GetID(), inShape1);
		uint part2 = sPartOf(mParts, inBody2.GetID(), inShape2);
		if (part1 == cNoPart || part2 == cNoPart)
			return true;
		bool listed = pair->second.mParts.find(swap? sKey(part2, part1) : sKey(part1, part2)) != pair->second.mParts.end();
		return listed == pair->second.mCollideList;
	}

	static constexpr uint	cNoPart = 0xffffffff;

private:
	struct Pair
	{
		bool				mCollideList = false;
		UnorderedSet<uint64> mParts;
	};

	static uint64			sKey(uint32 inA, uint32 inB)
	{
		return (uint64(inA) << 32) | uint64(inB);
	}

	/// The body pair's key (lower index and sequence number first); outSwap when inBody1 is the second
	static uint64			sBodyKey(const BodyID &inBody1, const BodyID &inBody2, bool &outSwap)
	{
		uint32 a = inBody1.GetIndexAndSequenceNumber(), b = inBody2.GetIndexAndSequenceNumber();
		outSwap = a > b;
		return outSwap? sKey(b, a) : sKey(a, b);
	}

	/// The part of a leaf shape of inBody: its user data is the collider index (a lone collider, wrapped in a
	/// rotated-translated shape whose user data is 0, is collider 0)
	static uint				sPartOf(const UnorderedMap<uint32, Array<uint>> &inParts, const BodyID &inBody, const Shape *inShape)
	{
		UnorderedMap<uint32, Array<uint>>::const_iterator parts = inParts.find(inBody.GetIndexAndSequenceNumber());
		if (parts == inParts.end())
			return cNoPart;
		uint64 collider = inShape->GetUserData();
		return collider < parts->second.size()? parts->second[size_t(collider)] : cNoPart;
	}

	UnorderedMap<uint32, Array<uint>> mParts;
	UnorderedMap<uint64, Pair> mPairs;
};
JPH_NAMESPACE_END
