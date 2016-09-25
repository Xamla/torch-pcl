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

PCLIMP(_Correspondences, Correspondences, getAt)(pcl::Correspondences *self, size_t pos)
{
  pcl::Correspondences &v = *self;
  const pcl::Correspondence &c = v[pos];
  _Correspondences c_ = { c.index_query, c.index_match, c.distance };
  return c_;
}

PCLIMP(void, Correspondences, setAt)(pcl::Correspondences *self, size_t pos, const _Correspondences &value)
{
  pcl::Correspondences &v = *self;
  v[pos] = pcl::Correspondence(value.index_query, value.index_match, value.distance);
}

PCLIMP(void, Correspondences, push_back)(pcl::Correspondences *self, const _Correspondences &value)
{
  self->push_back(pcl::Correspondence(value.index_query, value.index_match, value.distance));
}

PCLIMP(void, Correspondences, pop_back)(pcl::Correspondences *self)
{
  self->pop_back();
}

PCLIMP(void, Correspondences, clear)(pcl::Correspondences *self)
{
  self->clear();
}

PCLIMP(void, Correspondences, insert)(pcl::Correspondences *self, size_t pos, const _Correspondences &value)
{
  pcl::Correspondences &v = *self;
  pcl::Correspondences::iterator i = pos >= v.size() ? v.end() : v.begin() + pos;
  v.insert(i, pcl::Correspondence(value.index_query, value.index_match, value.distance));
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
