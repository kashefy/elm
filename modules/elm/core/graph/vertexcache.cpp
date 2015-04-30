#include "elm/core/graph/vertexcache.h"

#include "elm/core/exception.h"
#include "elm/core/debug_utils.h"

using namespace std;
using namespace elm;

VertexCache::~VertexCache()
{
}

VertexCache::VertexCache()
    : lim_(-1),
      capacity_(0)
{
}

void VertexCache::reserve(int capacity)
{
    descriptors_ = vector<VtxDescriptor >(capacity);
    ids_ = vector<VtxColor >(capacity, -1);
    capacity_ = capacity;
    lim_ = -1;
}

void VertexCache::remove(VtxColor vtx_id)
{
    if(ids_.size() > 0) {

        ids_[vtx_id] = -1;
        if(vtx_id == lim_) {

            while(lim_ >= 0 && ids_[lim_] >= 0) {

                lim_--;
            }
        }
    }
    else {

        ELM_THROW_KEY_ERROR("Cannot remove vertex from empty cache.");
    }
}

void VertexCache::clear()
{
    descriptors_.clear();
    ids_.clear();
    lim_ = -1;
}

bool VertexCache::validKey(VtxColor key) const
{
    return key <= lim_ &&
            key >= 0 &&
            key < capacity_;
}

bool VertexCache::isLink(VtxColor vtx_id) const
{
    if(validKey(vtx_id)) {

        VtxColor value = ids_[vtx_id];
        return isLinkValue(vtx_id, value) && !isMaskedValue(value);
    }
    else {

        return false;
    }
}

bool VertexCache::isLinkValue(VtxColor key, VtxColor value) const
{
    return key != value;
}

bool VertexCache::isMasked(VtxColor vtx_id) const
{
    if(validKey(vtx_id)) {

        return isMaskedValue(ids_[vtx_id]);
    }
    else {

        return false;
    }
}

bool VertexCache::isMaskedValue(VtxColor vtx_id_value) const
{
    return vtx_id_value < 0;
}

bool VertexCache::exists(VtxColor vtx_id) const
{
    if(validKey(vtx_id)) {

        VtxColor value = ids_[vtx_id];
        return !isLinkValue(vtx_id, value) && !isMaskedValue(value);
    }
    else {

        return false;
    }
    return !isLink(vtx_id);
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
    while(vtx_id >= capacity_) {

        descriptors_.push_back(VtxDescriptor());
        ids_.push_back(-1);
        capacity_++;
    }

    descriptors_[vtx_id] = descriptor;
    ids_[vtx_id] = vtx_id;
    if(vtx_id > lim_) {

        lim_ = vtx_id;
    }
}

void VertexCache::recordSubstitution(VtxColor src, VtxColor dst)
{
    bool is_src_key_valid = validKey(src);

    bool found_dst = !isMasked(dst);

    if(!found_dst) {

        stringstream s;
        s << "Destination (" << dst << ") does not exist.";
        ELM_THROW_KEY_ERROR(s.str());
    }
    else if(is_src_key_valid) {

        descriptors_[src] = descriptors_[dst];
        ids_[src] = ids_[dst];
    }
}

VtxColor VertexCache::Id(VtxColor vtx_id) const
{
    if(validKey(vtx_id)) {

        VtxColor value = ids_[vtx_id];
        if(!isMaskedValue(value)) {

            if(isLinkValue(vtx_id, value)) {

                return Id(value); // recursive call for multiple substitutions
            }
            else {
                return value;
            }
        }
        else {

            return value; // return mask
        }
    }
    else {
        ELM_THROW_KEY_ERROR("invalid id");
    }
}

