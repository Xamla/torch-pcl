#include "utils.h"

#define TYPE_KEY _

PCLIMP(IndicesVector_ptr *, IndicesVector, new)()
{
  return new IndicesVector_ptr(new std::vector<Indices_ptr>());
}

PCLIMP(void, IndicesVector, delete)(IndicesVector_ptr *self)
{
  delete self;
}

PCLIMP(unsigned int, IndicesVector, size)(IndicesVector_ptr *self)
{
  const std::vector<Indices_ptr>& indices = **self;
  return static_cast<unsigned int>(indices.size());
}

PCLIMP(void, IndicesVector, getAt)(IndicesVector_ptr *self, size_t pos, Indices_ptr *result)
{
  std::vector<Indices_ptr>& indices = **self;
  *result = indices[pos];
}

PCLIMP(void, IndicesVector, setAt)(IndicesVector_ptr *self, size_t pos, Indices_ptr *value)
{
  std::vector<Indices_ptr>& indices = **self;
  indices[pos] = *value;
}

PCLIMP(void, IndicesVector, push_back)(IndicesVector_ptr *self, Indices_ptr *value)
{
  std::vector<Indices_ptr>& indices = **self;
  indices.push_back(*value);
}

PCLIMP(void, IndicesVector, pop_back)(IndicesVector_ptr *self)
{
  std::vector<Indices_ptr>& indices = **self;
  indices.pop_back();
}

PCLIMP(void, IndicesVector, clear)(IndicesVector_ptr *self)
{
  std::vector<Indices_ptr>& indices = **self;
  indices.clear();
}

PCLIMP(void, IndicesVector, insert)(IndicesVector_ptr *self, size_t pos, Indices_ptr *value)
{
  std::vector<Indices_ptr>& indices = **self;
  std::vector<Indices_ptr>::iterator i = pos >= indices.size() ? indices.end() : indices.begin() + pos;
  indices.insert(i, *value);
}

PCLIMP(void, IndicesVector, erase)(IndicesVector_ptr *self, size_t begin, size_t end)
{
  if (begin >= end)
    return;
    
  std::vector<Indices_ptr>& indices = **self;
  std::vector<Indices_ptr>::iterator b = begin >= indices.size() ? indices.end() : indices.begin() + begin;
  std::vector<Indices_ptr>::iterator e = end >= indices.size() ? indices.end() : indices.begin() + end;
  indices.erase(b, e);
}

PCLIMP(bool, IndicesVector, empty)(IndicesVector_ptr *self)
{
  const std::vector<Indices_ptr>& indices = **self;
  return indices.empty();
}
