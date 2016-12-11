#include "utils.h"
#include <pcl/correspondence.h>

#define TYPE_KEY _

struct _Correspondences
{
  int index_query;
  int index_match;

  union
  {
    float distance;
    float weight;
  };
};


PCLIMP(Correspondences_ptr *, Correspondences, new)()
{
  return new Correspondences_ptr(new pcl::Correspondences());
}

PCLIMP(Correspondences_ptr *, Correspondences, clone)(Correspondences_ptr *self)
{
  return new Correspondences_ptr(new pcl::Correspondences(**self));
}

PCLIMP(void, Correspondences, delete)(Correspondences_ptr *self)
{
  delete self;
}

PCLIMP(int, Correspondences, size)(Correspondences_ptr *self)
{
  return static_cast<int>((*self)->size());
}

PCLIMP(_Correspondences, Correspondences, getAt)(Correspondences_ptr *self, size_t pos)
{
  pcl::Correspondences &v = **self;
  const pcl::Correspondence &c = v[pos];
  _Correspondences c_ = { c.index_query, c.index_match, c.distance };
  return c_;
}

PCLIMP(void, Correspondences, setAt)(Correspondences_ptr *self, size_t pos, const _Correspondences &value)
{
  pcl::Correspondences &v = **self;
  v[pos] = pcl::Correspondence(value.index_query, value.index_match, value.distance);
}

PCLIMP(void, Correspondences, push_back)(Correspondences_ptr *self, const _Correspondences &value)
{
  (*self)->push_back(pcl::Correspondence(value.index_query, value.index_match, value.distance));
}

PCLIMP(void, Correspondences, pop_back)(Correspondences_ptr *self)
{
  (*self)->pop_back();
}

PCLIMP(void, Correspondences, clear)(Correspondences_ptr *self)
{
  (*self)->clear();
}

PCLIMP(void, Correspondences, insert)(Correspondences_ptr *self, size_t pos, const _Correspondences &value)
{
  pcl::Correspondences &v = **self;
  pcl::Correspondences::iterator i = pos >= v.size() ? v.end() : v.begin() + pos;
  v.insert(i, pcl::Correspondence(value.index_query, value.index_match, value.distance));
}

PCLIMP(void, Correspondences, erase)(Correspondences_ptr *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;

  pcl::Correspondences& v = **self;
  pcl::Correspondences::iterator b = begin >= v.size() ? v.end() : v.begin() + begin;
  pcl::Correspondences::iterator e = end >= v.size() ? v.end() : v.begin() + end;
  v.erase(b, e);
}

PCLIMP(bool, Correspondences, empty)(Correspondences_ptr *self)
{
  return (*self)->empty();
}
