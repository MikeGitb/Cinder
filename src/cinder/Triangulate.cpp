/*
 Copyright (c) 2011, The Cinder Project: http://libcinder.org
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#include "cinder/Triangulate.h"
#include "cinder/Shape2d.h"
#include "../libtess2/tess.h"
#include "../libtess2/tesselator.h"

using namespace std;

namespace cinder {

template<int Dims>
class TessWrapper {
public:
	static_assert(Dims == 2 || Dims == 3, "");
	using vec = typename std::conditional<Dims == 2, vec2, vec3>::type;
	TessWrapper( int& allocatedMemCnt ) {
		mAlloc.memalloc = stdAlloc;
		mAlloc.memfree = stdFree;
		mAlloc.userData = &allocatedMemCnt;
		mAlloc.extraVertices = 2560; // realloc not provided, allow 2560 extra vertices XXX: where does this number come from.

		mTess = tessNewTess( &mAlloc );
		mTess->outOfMemory = 0;
		if ( !mTess )
			throw Triangulator::Exception();
	}

	~TessWrapper() {
		if ( mTess ) {
			tessDeleteTess( mTess );
		}
	}

	const vec*	getVertices()		{ return reinterpret_cast<const vec*>(::tessGetVertices( mTess )); }		//XXX Illigal cast?
	const uint32_t*	getElements()	{ return reinterpret_cast<const uint32_t*>(::tessGetElements( mTess ); }	
	int			getVertexCount()	{ return ::tessGetVertexCount( mTess ); }
	int			getElementCount()	{ return ::tessGetElementCount( mTess ); }

	int tesselate( int windingRule, int elementType, int polySize, const vec* normal )
	{
		return ::tessTesselate( mTess, windingRule, elementType, polySize, Dims, normal );
	}

	void addContour( const void* vertices, size_t stride, size_t numVertices)
	{
		::tessAddContour(mTess, Dims, vertices, (int) stride, (int) numVertices);
	}


private:
	static void* stdAlloc(void* userData, unsigned int size)
	{
		int *allocated = (int*)userData;
		*allocated += (int)size;
		return malloc(size);
	}
	static void stdFree(void* userData, void* ptr)
	{
		*((int*)userData) = 0;
		free(ptr);
	}

	TESStesselator* mTess = nullptr;
	TESSalloc mAlloc{};
};

Triangulator::Triangulator()
{
	mTess = std::unique_ptr<TessWrapper<2>>( new TessWrapper<2>(mAllocated) ); //c++14: mTess = std::make_unique<TessWrapper<2>>(mAllocated);
}

Triangulator::~Triangulator()
{
	//empty destructor defined in cpp file to allow forward declaration of TessObj
}

Triangulator::Triangulator( const Path2d &path, float approximationScale )
	: Triangulator()
{
	addPath( path, approximationScale );
}

Triangulator::Triangulator( const Shape2d &shape, float approximationScale )
	: Triangulator()
{
	addShape( shape, approximationScale );
}

Triangulator::Triangulator( const PolyLine2f &polyLine )
	: Triangulator()
{
	addPolyLine( polyLine );
}


void Triangulator::addShape( const Shape2d &shape, float approximationScale )
{
	for( auto& contour : shape.getContours() ) {
		addPath( contour, approximationScale );
	}
}

void Triangulator::addPath( const Path2d &path, float approximationScale )
{
	vector<vec2> subdivided = path.subdivide( approximationScale );
	mTess->addContour( subdivided.data(), sizeof( *subdivided.data() ), subdivided.size() );
}

void Triangulator::addPolyLine( const PolyLine2f &polyLine )
{
	if( polyLine.size() > 0 )
		mTess->addContour(polyLine.getPoints().data(), sizeof( *polyLine.getPoints().data() ), polyLine.size() );
}

void Triangulator::addPolyLine( const vec2 *points, size_t numPoints )
{
	if( numPoints > 0 )
		mTess->addContour(points, sizeof( *points ), numPoints );
}

TriMesh Triangulator::calcMesh( Winding winding )
{
	TriMesh result( TriMesh::Format().positions( 2 ) );

	if (mTess->tesselate((int)winding, TESS_POLYGONS, 3, nullptr)) {
		result.appendPositions(mTess->getVertices(), mTess->getVertexCount());
		result.appendIndices(mTess->getElements(), mTess->getElementCount() * 3);
	}
	return result;
}

TriMeshRef Triangulator::createMesh( Winding winding )
{
	auto result = make_shared<TriMesh>( TriMesh::Format().positions( 2 ) );

	if ( mTess->tesselate( (int)winding, TESS_POLYGONS, 3, nullptr) ) {
		result->appendPositions( mTess->getVertices(), mTess->getVertexCount() );
		result->appendIndices( mTess->getElements(), mTess->getElementCount() * 3 );
	}
	return result;
}

} // namespace cinder
