#include "elm/core/graph/vertexcache.h"

#include "elm/core/exception.h"
#include "elm/core/debug_utils.h"

using namespace std;
using namespace elm;

VertexCache::~VertexCache()
{
}

VertexCache::VertexCache()
    : lim_(-1)
{
}

VertexCache::VertexCache(int capacity)
{
    reserve(capacity);
}

void VertexCache::reserve(int capacity)
{
    descriptors_ = vector<VtxDescriptor >(capacity);
    ids_ = vector<VtxColor >(capacity, -1);
    lim_ = -1;
}

void VertexCache::remove(VtxColor vtx_id)
{
    ids_[vtx_id] = -1;
    if(vtx_id == lim_) {

        while(lim_ >= 0 && ids_[lim_] >= 0) {

            lim_--;
        }
    }
}

void VertexCache::clear()
{
    descriptors_.clear();
    ids_.clear();
    lim_ = -1;
}

bool VertexCache::exists(VtxColor vtx_id) const
{
    return vtx_id <= lim_ &&
            vtx_id >= 0 &&
            vtx_id < static_cast<int>(ids_.size()) &&
            ids_[vtx_id] >= 0;
}

bool VertexCache::find(VtxColor vtx_id, VtxDescriptor &vtx_descriptor) const
{
    bool found = exists(vtx_id);

    if(found) {

        vtx_descriptor = descriptors_[vtx_id]; // get cached descriptor
    }

    return found;
}

void VertexCache::insert(VtxColor vtx_id, const VtxDescriptor &descriptor)
{
    while(vtx_id >= static_cast<int>(descriptors_.size())) {

        descriptors_.push_back(VtxDescriptor());
        ids_.push_back(-1);
    }
    descriptors_[vtx_id] = descriptor;
    ids_[vtx_id] = vtx_id;
    if(vtx_id > lim_) {

        lim_ = vtx_id;
    }
}

void VertexCache::recordSubstitution(VtxColor src, VtxColor dst)
{
    bool found_src = src <= lim_ &&
            src >= 0 &&
            ids_[src] >= 0;

    bool found_dst = dst <= lim_ &&
            dst >= 0 &&
            ids_[dst] >= 0;

    if(found_src && found_dst) {

        descriptors_[src] = descriptors_[dst];
        ids_[src] = ids_[dst];
    }
}

