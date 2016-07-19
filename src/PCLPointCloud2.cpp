#define TYPE_KEY _
#include "utils.h"
#include <pcl/PCLPointCloud2.h>

PCLIMP(pcl::PCLPointCloud2*, PCLPointCloud2, new)()
{
  return new pcl::PCLPointCloud2();
}

PCLIMP(void, PCLPointCloud2, delete)(pcl::PCLPointCloud2 *self)
{
  delete self;
}

PCLIMP(void, PCLPointCloud2, getFieldNames)(pcl::PCLPointCloud2 *self, THByteStorage *string_buffer)
{
  std::string fieldNames;
  std::vector<pcl::PCLPointField>::const_iterator i = self->fields.begin();
  for (; i != self->fields.end(); ++i)
    fieldNames.append(i->name);
  stringToByteStorage(fieldNames, string_buffer);
}

PCLIMP(void, PCLPointCloud2, tostring)(pcl::PCLPointCloud2 *self, THByteStorage *string_buffer)
{
  std::stringstream ss;
  ss << *self;
  stringToByteStorage(ss.str(), string_buffer);
}
