#include "utils.h"
#include <pcl/correspondence.h>

#define TYPE_KEY _

PCLIMP(pcl::Correspondences *, Correspondences, new)()
{
  return new pcl::Correspondences();
}

PCLIMP(pcl::Correspondences *, Correspondences, clone)(pcl::Correspondences *self)
{
  return new pcl::Correspondences(*self);
}

PCLIMP(void, Correspondences, delete)(pcl::Correspondences *self)
{
  delete self;
}

PCLIMP(int, Correspondences, size)(pcl::Correspondences *self)
{
  return static_cast<int>(self->size());
}

PCLIMP(pcl::Correspondence &, Correspondences, at)(pcl::Correspondences *self, size_t pos)
{
  pcl::Correspondences& v = *self;
  return v[pos];
}

PCLIMP(void, Correspondences, push_back)(pcl::Correspondences *self, const pcl::Correspondence &value)
{
  self->push_back(value);
}

PCLIMP(void, Correspondences, pop_back)(pcl::Correspondences *self)
{
  self->pop_back();
}

PCLIMP(void, Correspondences, clear)(pcl::Correspondences *self)
{
  self->clear();
}

PCLIMP(void, Correspondences, insert)(pcl::Correspondences *self, size_t pos, const pcl::Correspondence &value)
{
  pcl::Correspondences& v = *self;
  pcl::Correspondences::iterator i = pos >= v.size() ? v.end() : v.begin() + pos;
  v.insert(i, value);
}

PCLIMP(void, Correspondences, erase)(pcl::Correspondences *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;
    
  pcl::Correspondences& v = *self;
  pcl::Correspondences::iterator b = begin >= v.size() ? v.end() : v.begin() + begin;
  pcl::Correspondences::iterator e = end >= v.size() ? v.end() : v.begin() + end;
  v.erase(b, e);
}

PCLIMP(bool, Correspondences, empty)(pcl::Correspondences *self)
{
  return self->empty();
}
